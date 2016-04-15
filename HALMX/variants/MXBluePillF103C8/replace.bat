@echo off &setlocal
setlocal enabledelayedexpansion

set "search=%1"
set "replace=%2"
set "textfile=Src\_spi.c"
set "newfile=Src\__spi.c"
set "textfile2=_spi.c"
(for /f "delims=" %%i in (%textfile%) do (
    set "line=%%i"
    set "line=!line:%search%=%replace%!"
    echo(!line!
))>"%newfile%"
del %textfile%
rename %newfile%  %textfile2%
endlocal