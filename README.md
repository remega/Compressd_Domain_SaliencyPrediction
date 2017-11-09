Compressd_Domain_SaliencyPrediction
==========
The released code for paper "Learning to Detect Video Saliency with HEVC Features" in TIP2017 ([link](http://ieeexplore.ieee.org/abstract/document/7742914/)), from Lai Jiang, Mai Xu in Beihang University(2016). 

## Abstract
Saliency detection has been widely studied to predict human fixations, with various applications in computer vision and image processing.  For saliency detection, we argue in this paper that the state-of-the-art high efficiency video coding (HEVC) standard can be used to generate the useful features in compressed domain. Therefore, this paper proposes to learn the video saliency model, with regard to HEVC features. First, we establish an eye tracking database for video saliency detectio. Through the statistical analysis on our eye tracking database, we find out that human fixations tend to fall into the regions with large-valued HEVC features on splitting depth, bit allocation, and motion vector (MV). In addition, three observations are obtained with the further analysis on our eye tracking database. Accordingly, several features in HEVC domain are proposed on the basis of splitting depth, bit allocation, and MV. Next, a kind of support vector machine (SVM) is learned to integrate those HEVC features together, for video saliency detection.
Since almost all video data are stored in the compressed form, our method is able to avoid both the computational cost on decoding and the storage cost on raw data. More importantly, experimental results show that the proposed method is superior to other state-of-the-art saliency detection methods, either in compressed or uncompressed domain.

![](/figs/fig1.png)

## Publication
Our work is published in [TIP2017](http://ieeexplore.ieee.org/abstract/document/7742914/), one can cite with the Bibtex code:  

```
@article{xu2017learning,
  title={Learning to detect video saliency with HEVC features},
  author={Xu, Mai and Jiang, Lai and Sun, Xiaoyan and Ye, Zhaoting and Wang, Zulin},
  journal={IEEE Transactions on Image Processing},
  volume={26},
  number={1},
  pages={369--385},
  year={2017},
  publisher={IEEE}
}
```

## Models
Nine designed compressed domain features followed a learned SVM.

![Features](/figs/compresseddomain.png "Features")
![SVM](/figs/svm.png "SVM")

## Usage
To  apply our method, we first encode the target video, and then extract compressed domain features. At last, generate the saliency maps.

### Software
1.	Matlab 2012 (or later)

2.	Visual studio 2010 (or later)

### Encode the video
In paper, video is encoded by **HM16.0 (HEVC)** in rate control mode, with the bit-rates of the same as those at fixed QP=37. However, any public encoder with different settings can be applied to extract the compressed domain features, such as **X265 in ffmpeg**. Here, we give two examples.

#### HM16.0 (HEVC)
Videos can be encoded in the HECV format by HM16.0, which can be downloaded in this  [link](https://hevc.hhi.fraunhofer.de/svn/svn_HEVCSoftware/tags/). The video is first encoded in fixed QP mode to obtain the corresponding bit-rates. Then, video is encoded in rate control mode to generate the final bit stream file (.bin).	

    1)	Transform the video to YUV format (if not). Run trans2yuv.bat.

    2)	Move the yuv file to .\HM16.0_fixqp\bin\vc10\Win32\Debug\. 

    3)	Modify .\HM16.0_fixqp\bin\vc10\Win32\Debug\VideoInfo.cfg according to your video information.

    4)	Modify .\HM16.0_fixqp\bin\vc10\Win32\Debug\encoder_lowdelay_P_main.cfg, mainly in Quantization_QP=X (37 is recommended), Rate Control_RateControl=0.

    5)	Run the project in .\HM16.0_fixqp\build\HM_vc10.sln. Record the bit-rates in the screen when the project is finished. 

    6)	Modify .\HM16.0_fixqp\bin\vc10\Win32\Debug\encoder_lowdelay_P_main.cfg, mainly in, Rate Control_RateControl=1, Rate Control_RateControl= TargetBitrate=X (the recorded bit-rates).

    7)	Run the project in .\HM16.0_fixqp\build\HM_vc10.sln again. The bit stream file str.bin is in .\HM16.0_fixqp\bin\vc10\Win32\Debug\. 

#### ffmpeg (X265)

Transcode the video to bit stream file (.hevc) by the x265 encoder in ffmpeg3.2.2, with QP set to 37.
Run x265encode.bat

2.Feature extraction&Generate the saliency map

Normal version (in paper)
 
1)	Mex the cpp file in matlab: Mex computecontrast5.cpp

2)	Run Main.m (modify the input/output on your own)

Fast version

1)	Move the bit stream file (str.bin) to .\HM_16.0_features\bin\vc10.

2)	Modify the video information in TLibDecoder_ TDecCu.h and TLibDecoder_TDecGop.cpp

3)	Link your OpenCV 

4)	Run the project in .\HM_16.0_features\build\HM_vc10.sln (Release X64 default)

IF any question, please contact jianglai.china@aliyun.com.
