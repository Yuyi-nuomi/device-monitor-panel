$enc = [System.Text.UTF8Encoding]::new($false)
$path = [System.IO.Path]::GetFullPath("PCT_100.ino")
$content = [System.IO.File]::ReadAllText($path, $enc)

# Fix: add newline between setRGB(0,0,0) and closing brace
$content = [regex]::Replace($content,
  '(  setRGB\(0, 0, 0\);\s*)(}\s+)',
  '$1' + "`n" + '$2'
)

Write-Host "Fixed newline before rgbBootBlink closing brace"

[System.IO.File]::WriteAllText($path, $content, $enc)
Write-Host "Written"

# Verify
$lines = $content -split "`n"
for ($idx=84; $idx -le 90; $idx++) {
  Write-Host "L${idx}: $($lines[$idx])"
}
