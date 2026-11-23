param([string]$board, [string]$tag, [string]$view = "", [double]$scale = 16)
$g = 'C:\Claude_projects\GridLess\target\release\gridless.exe'
$a = @()
if ($view) { $a += @('--view', $view) }
& $g render $board --png "$tag`_FB.png" --no-fills --scale $scale @a 2>&1 | Out-Null
& $g render $board --png "$tag`_F.png" --layers F.Cu --no-fills --scale $scale @a 2>&1 | Out-Null
& $g render $board --png "$tag`_B.png" --layers B.Cu --no-fills --scale $scale @a 2>&1 | Out-Null
