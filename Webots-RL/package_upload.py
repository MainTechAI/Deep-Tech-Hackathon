"""Package the reviewed experiment for browser upload to LUMI."""
from pathlib import Path
from zipfile import ZipFile, ZIP_DEFLATED

root = Path(__file__).resolve().parent
output = root / "runs" / "lumi-upload.zip"
output.parent.mkdir(exist_ok=True)
with ZipFile(output, "w", ZIP_DEFLATED) as archive:
    for path in sorted(root.rglob("*")):
        relative = path.relative_to(root)
        if path.is_file() and not any(p in ("runs", "__pycache__") for p in relative.parts):
            archive.write(path, Path("Webots-RL") / relative)
print(output)
