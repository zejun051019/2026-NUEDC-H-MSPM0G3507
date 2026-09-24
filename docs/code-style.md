# Code style and formatting

## Conventions

- Application and test code use C11, four-space indentation, UTF-8 without a BOM, and LF line endings.
- `.clang-format` is the repository formatting source; `.editorconfig` and `.gitattributes` define editor and Git text-file behavior.
- The pinned formatter used for this cleanup is `clang-format` 18.1.8.
- Formatting scope is all first-party C/H under `user/` and `tests/`, including Q3. Q3 algorithms, calibration values, control parameters and sequencing remain behavior-protected; style-only changes are allowed. TI/CMSIS vendor code and SysConfig-generated files are not reformatted.
- Preserve vendor copyright and license headers. Formatting is not permission to modify vendor code.

## Check or apply formatting

Create an ignored, project-local formatter environment and run from the repository root:

```powershell
python -m venv .venv-format
.\.venv-format\Scripts\python.exe -m pip install clang-format==18.1.8
$env:PATH = "$PWD\.venv-format\Scripts;$env:PATH"
powershell -ExecutionPolicy Bypass -File tools/check_format.ps1
```

To apply the repository style to the maintained source scope:

```powershell
powershell -ExecutionPolicy Bypass -File tools/check_format.ps1 -Fix
```

Review the resulting diff after applying formatting. The formatter must not change control expressions, constants, initialization order, or generated code. For Q3, review each changed hunk as layout-only; preprocessor line-continuation formatting can make Git's whitespace-ignoring diff still display a hunk.
