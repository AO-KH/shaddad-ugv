# pdf_pages_to_png.ps1 - render pages of a PDF to PNG with the Windows PDF renderer (Windows.Data.Pdf, no installs),
# for checking a built report page by page.
# usage: powershell -ExecutionPolicy Bypass -File pdf_pages_to_png.ps1 -Pdf <abs.pdf> -OutDir <abs dir> -Pages 1,4,9 [-Width 1100]
param(
    [Parameter(Mandatory = $true)][string]$Pdf,
    [Parameter(Mandatory = $true)][string]$OutDir,
    [string]$Pages = '1',
    [int]$Width = 1100
)
$ErrorActionPreference = 'Stop'
Add-Type -AssemblyName System.Runtime.WindowsRuntime
$ext = [System.WindowsRuntimeSystemExtensions].GetMethods()
$asTaskOp = ($ext | Where-Object { $_.Name -eq 'AsTask' -and $_.GetParameters().Count -eq 1 -and
        $_.GetParameters()[0].ParameterType.Name -eq 'IAsyncOperation`1' })[0]
$asTaskAction = ($ext | Where-Object { $_.Name -eq 'AsTask' -and $_.GetParameters().Count -eq 1 -and
        $_.GetParameters()[0].ParameterType.Name -eq 'IAsyncAction' })[0]
function Await($op, [Type]$t) {
    $task = $asTaskOp.MakeGenericMethod($t).Invoke($null, @($op))
    $task.Wait(-1) | Out-Null
    return $task.Result
}
function AwaitAction($action) {
    $task = $asTaskAction.Invoke($null, @($action))
    $task.Wait(-1) | Out-Null
}
[Windows.Storage.StorageFile, Windows.Storage, ContentType = WindowsRuntime] | Out-Null
[Windows.Storage.StorageFolder, Windows.Storage, ContentType = WindowsRuntime] | Out-Null
[Windows.Data.Pdf.PdfDocument, Windows.Data.Pdf, ContentType = WindowsRuntime] | Out-Null
[Windows.Data.Pdf.PdfPageRenderOptions, Windows.Data.Pdf, ContentType = WindowsRuntime] | Out-Null
[Windows.Storage.Streams.IRandomAccessStream, Windows.Storage.Streams, ContentType = WindowsRuntime] | Out-Null

New-Item -ItemType Directory -Force -Path $OutDir | Out-Null
$file = Await ([Windows.Storage.StorageFile]::GetFileFromPathAsync($Pdf)) ([Windows.Storage.StorageFile])
$doc = Await ([Windows.Data.Pdf.PdfDocument]::LoadFromFileAsync($file)) ([Windows.Data.Pdf.PdfDocument])
$folder = Await ([Windows.Storage.StorageFolder]::GetFolderFromPathAsync($OutDir)) ([Windows.Storage.StorageFolder])
$base = [System.IO.Path]::GetFileNameWithoutExtension($Pdf)
$list = @($Pages -split '[,; ]+' | Where-Object { $_ } | ForEach-Object { [int]$_ })
foreach ($p in $list) {
    if ($p -lt 1 -or $p -gt $doc.PageCount) { continue }
    $page = $doc.GetPage([uint32]($p - 1))
    $name = "{0}_p{1:D2}.png" -f $base, $p
    $sf = Await ($folder.CreateFileAsync($name, [Windows.Storage.CreationCollisionOption]::ReplaceExisting)) ([Windows.Storage.StorageFile])
    $stream = Await ($sf.OpenAsync([Windows.Storage.FileAccessMode]::ReadWrite)) ([Windows.Storage.Streams.IRandomAccessStream])
    $opts = New-Object Windows.Data.Pdf.PdfPageRenderOptions
    $opts.DestinationWidth = [uint32]$Width
    AwaitAction ($page.RenderToStreamAsync($stream, $opts))
    $stream.Dispose()
    $page.Dispose()
    Write-Output (Join-Path $OutDir $name)
}
Write-Output ("pages in the PDF: {0}" -f $doc.PageCount)
