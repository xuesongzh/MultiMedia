#! /bin/bash
rm -rf build
mkdir build
cd build
cmake -DCMAKE_BUILD_TYPE=Debug ..
make -j8
# ./cmd_dir .
# ./video_encoder ~/Video/input_1280x720.yuv output.h264 libx264
# ./video_decoder ~/Video/es.h264 output.yuv
# ./audio_decoder ~/Video/test.mp3 output.pcm MP3
# ./audio_encoder ~/Video/input_f32le_2_44100.pcm output.mp3 MP3
# ./demuxer ~/Video/test.mp4 output1.yuv output2.pcm
# ./muxer ~/Video/es.h264 ~/Video/test.mp3 output.mp4
# ./video_filter ~/Video/input_1280x720.yuv 1280 720 20 hflip filtered.yuv
# ./audio_filter ~/Video/input_f32le_2_44100.pcm output.pcm 0.5
# ./video_transformer ~/Video/input_1280x720.yuv 1280x720 YUV420P scaled.data 640x480 RGB24
./audio_resampler ~/Video/input_f32le_2_44100.pcm 44100 fltp STEREO resampled.pcm 22050 s16 MONO