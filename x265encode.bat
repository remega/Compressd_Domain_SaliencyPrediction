
for /r .\videos\ %%k in (*.avi) do .\ffmpeg\bin\ffmpeg.exe -fflags +genpts -i %%~k -vcodec libx265 -x265-params b-adapt=0:bframes=0:qp=37:keyint=-1 -y .\HEVCfiles\%%~nk\str.hevc