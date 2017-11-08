
for /r .\videos\ %%k in (*.avi) do .\ffmpeg\bin\ffmpeg.exe -fflags +genpts -i %%k -y .\HEVCfiles\%%~nk\%%~nk.yuv