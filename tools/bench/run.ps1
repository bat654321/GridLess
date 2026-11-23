param([string]$tag, [string]$cfg = "", [string]$extra = "")
# route bare board -> $tag.kicad_pcb, KiCad DRC, renders
$ErrorActionPreference = 'Continue'
$g = 'C:\Claude_projects\GridLess\target\release\gridless.exe'
$cli = 'C:\Program Files\KiCad\10.0\bin\kicad-cli.exe'
Copy-Item bare.kicad_pro "$tag.kicad_pro" -Force
Copy-Item bare.kicad_dru "$tag.kicad_dru" -Force
$a = @('route', 'bare.kicad_pcb', '-o', "$tag.kicad_pcb", '--project', 'bare.kicad_pro')
if ($cfg) { $a += @('-c', $cfg) } else { $a += @('-c', 'empty.toml') }
if ($env:GL_EXTRA) { $a += $env:GL_EXTRA.Split(' ') }
$t0 = Get-Date
& $g @a 2>&1 | ForEach-Object { "$_" } | Set-Content "$tag.log"
$sec = ((Get-Date) - $t0).TotalSeconds
Get-Content "$tag.log" | Select-String -Pattern 'done in|topology:|finishing|DRC:' | ForEach-Object { $_.Line }
& $cli pcb drc --format json --units mm --severity-error --refill-zones --save-board -o "$tag.drc.json" "$tag.kicad_pcb" 2>&1 | Out-Null
python drcsum.py "$tag.drc.json"
python netstat.py "$tag.kicad_pcb" | Select-Object -First 1
"time {0:N0} s" -f $sec
& $g render "$tag.kicad_pcb" --png "$tag`_FB.png" --no-fills --scale 16 2>&1 | Out-Null
& $g render "$tag.kicad_pcb" --png "$tag`_R.png" --no-fills --scale 40 --view "164,84,189,114" 2>&1 | Out-Null
