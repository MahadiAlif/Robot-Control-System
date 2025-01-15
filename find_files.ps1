# find_files.ps1
$backupPath = "e:\TEAM_ROBOTO\project_backup"
$files = Get-ChildItem -Path $backupPath -Include "*.c", "*.h" -Recurse
$output = foreach ($f in $files) {
    $relative = $f.FullName.Substring($backupPath.Length + 1)
    "$relative"
}
$output | Out-File -FilePath "e:\TEAM_ROBOTO\backup_files.txt" -Force
Write-Host "Found $($files.Count) C files and saved to backup_files.txt"
