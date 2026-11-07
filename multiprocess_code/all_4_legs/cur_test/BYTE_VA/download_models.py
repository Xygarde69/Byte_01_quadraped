# ════════════════════════════════════════════════════════════════════════
# REPO MAP  ·  multiprocess_code/all_4_legs/cur_test/BYTE_VA/download_models.py
# STATUS    ·  PROTOTYPE · voice assistant (BYTE_VA)
# ────────────────────────────────────────────────────────────────────────
# One-time setup: downloads silero_vad.onnx and the Zipformer KWS model
# into models/, writes keywords_raw.txt and tokenizes it to
# keywords.txt.
# ────────────────────────────────────────────────────────────────────────
# RUN       ·  python3 download_models.py
# ════════════════════════════════════════════════════════════════════════
"""
download_models.py
------------------
One-time setup script for BYTE.

Steps performed
~~~~~~~~~~~~~~~
1. Download silero_vad.onnx        → models/
2. Download & extract Zipformer KWS tarball → models/
3. Write keywords_raw.txt          (human-readable wake-word definitions)
4. Tokenize keywords (sherpa_onnx text2token API) → keywords.txt  (BPE-tokenized)

Run this script once (with the byte_env venv active) before starting app.py:

    python download_models.py
"""

import subprocess
import sys
import tarfile
from pathlib import Path

# Force UTF-8 output on Windows (avoids cp1252 UnicodeEncodeError)
if hasattr(sys.stdout, "reconfigure"):
    sys.stdout.reconfigure(encoding="utf-8")

import requests

# --------------------------------------------------------------------------- #
# Paths
# --------------------------------------------------------------------------- #
ROOT       = Path(__file__).parent
MODELS_DIR = ROOT / "models"
KWS_DIR    = MODELS_DIR / "sherpa-onnx-kws-zipformer-gigaspeech-3.3M-2024-01-01"

KEYWORDS_RAW = ROOT / "keywords_raw.txt"
KEYWORDS_OUT = ROOT / "keywords.txt"

# --------------------------------------------------------------------------- #
# Download URLs  (sherpa-onnx official GitHub releases)
# --------------------------------------------------------------------------- #
_BASE = "https://github.com/k2-fsa/sherpa-onnx/releases/download"

VAD_URL = (
    f"{_BASE}/asr-models/silero_vad.onnx"
)
KWS_URL = (
    f"{_BASE}/kws-models/"
    "sherpa-onnx-kws-zipformer-gigaspeech-3.3M-2024-01-01.tar.bz2"
)

# --------------------------------------------------------------------------- #
# Helpers
# --------------------------------------------------------------------------- #
def _download(url: str, dest: Path) -> None:
    """Stream-download *url* to *dest*, printing a progress indicator."""
    if dest.exists():
        print(f"  [skip] {dest.name} already exists.")
        return

    print(f"  Downloading {dest.name} ...", end="", flush=True)
    dest.parent.mkdir(parents=True, exist_ok=True)

    with requests.get(url, stream=True, timeout=120) as resp:
        resp.raise_for_status()
        total = int(resp.headers.get("content-length", 0))
        downloaded = 0
        with open(dest, "wb") as fh:
            for chunk in resp.iter_content(chunk_size=65_536):
                fh.write(chunk)
                downloaded += len(chunk)
                if total:
                    pct = downloaded * 100 // total
                    print(f"\r  Downloading {dest.name} ... {pct:3d}%",
                          end="", flush=True)
    print(f"\r  Downloaded  {dest.name}  [OK]")


def _extract_tar(archive: Path, dest_dir: Path) -> None:
    """Extract a .tar.bz2 archive, skipping if already extracted."""
    if dest_dir.exists():
        print(f"  [skip] {dest_dir.name}/ already extracted.")
        return
    print(f"  Extracting {archive.name} ...", flush=True)
    with tarfile.open(archive, "r:bz2") as tf:
        tf.extractall(dest_dir.parent)
    print(f"  Extracted  {dest_dir.name}/  [OK]")


# --------------------------------------------------------------------------- #
# Step 1 & 2 – Download models
# --------------------------------------------------------------------------- #
def download_models() -> None:
    print("\n-- Step 1 / 2 : Downloading models ---------------------")
    MODELS_DIR.mkdir(parents=True, exist_ok=True)

    # Silero VAD
    vad_dest = MODELS_DIR / "silero_vad.onnx"
    _download(VAD_URL, vad_dest)

    # Zipformer KWS tarball
    kws_tar = MODELS_DIR / "sherpa-onnx-kws-zipformer-gigaspeech-3.3M-2024-01-01.tar.bz2"
    _download(KWS_URL, kws_tar)
    _extract_tar(kws_tar, KWS_DIR)


# --------------------------------------------------------------------------- #
# Step 3 – Write keywords_raw.txt
# --------------------------------------------------------------------------- #
def write_keywords_raw() -> None:
    print("\n-- Step 3 : Writing keywords_raw.txt -------------------")
    if KEYWORDS_RAW.exists():
        print(f"  [skip] {KEYWORDS_RAW.name} already exists.")
        return

    # Format: KEYWORD_PHRASE :boost_score #threshold
    # Using multi-syllable phrases for better discrimination.
    keywords = (
        "HEY BITE :1.5 #0.05\n"
        "HELLO BITE :1.5 #0.05\n"
    )
    KEYWORDS_RAW.write_text(keywords, encoding="utf-8")
    print(f"  Written: {KEYWORDS_RAW}")
    print(f"  Contents:\n{keywords.rstrip()}")


# --------------------------------------------------------------------------- #
# Step 4 – Tokenize keywords using the sherpa_onnx Python API directly
#           (avoids the CLI's heavy import chain for non-BPE token types)
# --------------------------------------------------------------------------- #
def tokenize_keywords() -> None:
    print("\n-- Step 4 : Tokenizing keywords (Python API) -------------------")

    tokens_path = KWS_DIR / "tokens.txt"
    bpe_path    = KWS_DIR / "bpe.model"

    for p in (tokens_path, bpe_path):
        if not p.exists():
            sys.exit(
                f"\n[ERROR] Expected model file not found: {p}\n"
                "        Did Step 2 (model download & extraction) succeed?\n"
                "        Re-run this script and check your internet connection."
            )

    # Read raw keyword lines (strip comments / blank lines for the tokenizer,
    # but we need to keep the :score #threshold suffixes for the output).
    raw_lines = KEYWORDS_RAW.read_text(encoding="utf-8").splitlines()
    raw_lines = [l for l in raw_lines if l.strip()]

    # Split each line into "PHRASE :score #threshold" parts.
    # Only the PHRASE part is tokenized; the suffixes are re-appended.
    phrases, suffixes = [], []
    for line in raw_lines:
        # e.g.  "HEY BITE :1.5 #0.35"
        parts = line.split(":")
        phrases.append(parts[0].strip())
        suffixes.append((":" + ":".join(parts[1:])).strip() if len(parts) > 1 else "")

    try:
        from sherpa_onnx.utils import text2token
    except ImportError:
        sys.exit(
            "\n[ERROR] Could not import sherpa_onnx.utils.text2token.\n"
            "        Make sure sherpa-onnx is installed in the active venv:\n"
            "            pip install -r requirements.txt\n"
        )

    print(f"  Tokenizing {len(phrases)} phrase(s) with BPE model ...", flush=True)
    try:
        token_lists = text2token(
            phrases,
            tokens=str(tokens_path),
            tokens_type="bpe",
            bpe_model=str(bpe_path),
        )
    except Exception as exc:
        sys.exit(f"\n[ERROR] Tokenization failed: {exc}\n")

    # Write keywords.txt in the format KeywordSpotter expects:
    #   TOKEN1 TOKEN2 ... :score #threshold
    out_lines = []
    for token_list, suffix in zip(token_lists, suffixes):
        token_str = " ".join(token_list)
        out_lines.append(f"{token_str} {suffix}".strip())

    KEYWORDS_OUT.write_text("\n".join(out_lines) + "\n", encoding="utf-8")

    print(f"  Generated: {KEYWORDS_OUT}")
    print(f"  Contents:\n{KEYWORDS_OUT.read_text(encoding='utf-8').rstrip()}")


# --------------------------------------------------------------------------- #
# Main
# --------------------------------------------------------------------------- #
def main() -> None:
    print("=" * 60)
    print("  BYTE – Model Setup")
    print("=" * 60)

    download_models()
    write_keywords_raw()
    tokenize_keywords()

    print("\n" + "=" * 60)
    print("  Setup complete!  Run: python app.py")
    print("=" * 60)


if __name__ == "__main__":
    main()  