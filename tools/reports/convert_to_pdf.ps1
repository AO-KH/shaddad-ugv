# convert_to_pdf.ps1 - open the built report in Microsoft Word, update the TOC and all fields,
# save the docx (stores the filled TOC) and export a PDF with heading bookmarks.
param(
    [Parameter(Mandatory = $true)][string]$Docx,
    [Parameter(Mandatory = $true)][string]$Pdf
)
$ErrorActionPreference = 'Stop'
$w = New-Object -ComObject Word.Application
$w.Visible = $false
$w.DisplayAlerts = 0
try {
    $d = $w.Documents.Open($Docx, $false, $false, $false)
    $d.Repaginate()
    $d.TablesOfContents | ForEach-Object { $_.Update() }
    $d.Fields.Update() | Out-Null
    $d.Repaginate()
    $d.TablesOfContents | ForEach-Object { $_.UpdatePageNumbers() }
    $d.Save()
    # ExportAsFixedFormat(path, wdExportFormatPDF=17, OpenAfterExport, OptimizeFor=print(0), Range=all(0),
    #   From, To, Item=content(0), IncludeDocProps, KeepIRM, CreateBookmarks=headings(1), DocStructureTags, BitmapMissingFonts, UseISO19005_1)
    $d.ExportAsFixedFormat($Pdf, 17, $false, 0, 0, 1, 1, 0, $true, $true, 1, $true, $true, $false)
    $pages = $d.ComputeStatistics(2)
    Write-Output "PDF written: $Pdf ($pages pages)"
    $d.Close(0)
}
finally {
    $w.Quit()
    [System.Runtime.InteropServices.Marshal]::ReleaseComObject($w) | Out-Null
}
