
The released code for paper "Learning to Detect Video Saliency with HEVC Features" in TIP2016, from Lai Jiang, Mai Xu in Beihang University(2016). 
Software
1.	Matlab 2012 (or later)
2.	Visual studio 2010 (or later)
Usage
1. Encode the video.
In paper, video is encoded by HM16.0 (HEVC) in rate control mode, with the bit-rates of the same as those at fixed QP=37. However, any public encoder with different settings can be applied to extract the compressed domain features. Here, we give two examples.
	HM16.0 (HEVC)
Videos can be encoded in the HECV format by HM16.0, which can be downloaded in https://hevc.hhi.fraunhofer.de/svn/svn_HEVCSoftware/tags/. The video is first encoded in fixed QP mode to obtain the corresponding bit-rates. Then, video is encoded in rate control mode to generate the final bit stream file (.bin).	
1)	Transform the video to YUV format (if not). Run trans2yuv.bat.
2)	Move the yuv file to .\HM16.0_fixqp\bin\vc10\Win32\Debug\. 
3)	Modify .\HM16.0_fixqp\bin\vc10\Win32\Debug\VideoInfo.cfg according to your video information.
4)	Modify .\HM16.0_fixqp\bin\vc10\Win32\Debug\encoder_lowdelay_P_main.cfg, mainly in Quantization_QP=X (37 is recommended), Rate Control_RateControl=0.
5)	Run the project in .\HM16.0_fixqp\build\HM_vc10.sln. Record the bit-rates in the screen when the project is finished. 
6)	Modify .\HM16.0_fixqp\bin\vc10\Win32\Debug\encoder_lowdelay_P_main.cfg, mainly in, Rate Control_RateControl=1, Rate Control_RateControl= TargetBitrate=X (the recorded bit-rates).
7)	Run the project in .\HM16.0_fixqp\build\HM_vc10.sln again. The bit stream file str.bin is in .\HM16.0_fixqp\bin\vc10\Win32\Debug\. 
	ffmpeg (X265)
Transcode the video to bit stream file (.hevc) by the x265 encoder in ffmpeg3.2.2, with QP set to 37.
Run x265encode.bat

2.Feature extraction&Generate the saliency map
	 Normal version (in paper)
1)	Mex the cpp file in matlab: Mex computecontrast5.cpp
2)	Run Main.m (modify the input/output on your own)
	Fast version
1)	Move the bit stream file (str.bin) to .\HM_16.0_features\bin\vc10.
2)	Modify the video information in TLibDecoder_ TDecCu.h and TLibDecoder_TDecGop.cpp
3)	Link your OpenCV 
4)	Run the project in .\HM_16.0_features\build\HM_vc10.sln (Release X64 default)



IF any question, please contact jianglai.china@aliyun.com.
