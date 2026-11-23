$tags = $args
# run several configs in parallel: tag N uses N.toml (e.g. t05 -> t05.toml)
$ps = foreach ($t in $tags) { Start-Process powershell -ArgumentList "-ExecutionPolicy Bypass -File run.ps1 $t $t.toml" -RedirectStandardOutput "$t.out" -NoNewWindow -PassThru }
$ps | Wait-Process
foreach ($t in $tags) { "== $t"; Get-Content "$t.out" | Select-String -NotMatch 'topology|finishing' }
