#include <stdio.h>
#include <stdlib.h>
#include <io.h>
#include <fcntl.h>

#include "modules/audio_processing/aec/echo_cancellation.h"
#include "modules/audio_processing/audio_buffer.h"
#include "modules/audio_processing/agc/legacy/gain_control.h"
#include "modules/audio_processing/ns/noise_suppression.h"
#include "common_audio/vad/include/webrtc_vad.h"

#pragma warning(disable:4996)

#undef max

void GenerateFloatFrame(short * data, int n, float * const * float_frames)
{
    for (int i = 0; i < n; i++) {
#if 0
        float_frames[0][i] = static_cast<float>(data[i]) / 
            static_cast<float>(std::numeric_limits<short>::max());
        if (fabs(float_frames[0][i]) > 1) {
            printf("invalid point\n");
            break;
        }
#else
        float_frames[0][i] = static_cast<float>(data[i]);
#endif
    }
}

void GenerateShortFrame(float * const * float_frames, short * data, int n)
{
    for (int i = 0; i < n; i++) {
#if 0
        data[i] = (short)((static_cast<float>(std::numeric_limits<short>::max()) *
            float_frames[0][i]));
#else
        data[i] = static_cast<short>(float_frames[0][i] + 0.5);
#endif
    }
}

static void AEC(void)
{
    printf("test aec begin\n");

    int fd_far = open("../speaker.pcm", O_RDONLY | O_BINARY);
    int fd_near = open("../micin.pcm", O_RDONLY | O_BINARY);
    int fd_out = open("../out.pcm", O_WRONLY | O_BINARY | O_CREAT | O_TRUNC, 0666);

    void * aec = webrtc::WebRtcAec_Create();
    int ret = webrtc::WebRtcAec_Init(aec, 8000, 8000);
    printf("WebRtcAec_Init result:%d\n", ret);

    webrtc::AecConfig config;
    config.nlpMode = webrtc::kAecNlpConservative;
    webrtc::WebRtcAec_set_config(aec, config);

#define WND_SIZE 80
    short far_frame[WND_SIZE], near_frame[WND_SIZE], out_frame[WND_SIZE];
    int frame_size = WND_SIZE * 2;
    //webrtc::AudioBuffer fb(WND_SIZE, 1, WND_SIZE, 2, WND_SIZE);

    std::array<float, 480> float_frame1;
    std::array<float, 480> float_frame2;

    memset(&float_frame1[0], 0, 480 * sizeof(float));
    memset(&float_frame2[0], 0, 480 * sizeof(float));

    std::array<float * const, 2> float_frame_ptrs = {
        &float_frame1[0], &float_frame2[0],
    };

    float * const * ptr_to_float_frame = &float_frame_ptrs[0];
    int farcount = 0;

    do {
        int nfar = read(fd_far, far_frame, frame_size);
        if (nfar != frame_size) {
            break;
        }


        int nnear = read(fd_near, near_frame, frame_size);
        if (nfar != frame_size || nnear != frame_size) {
            break;
        }

        GenerateFloatFrame(far_frame, WND_SIZE, ptr_to_float_frame);
        int ret = webrtc::WebRtcAec_BufferFarend(aec, ptr_to_float_frame[0], WND_SIZE);
        //printf("WebRtcAec_BufferFarend:%d\n", ret);

        GenerateFloatFrame(near_frame, WND_SIZE, ptr_to_float_frame);
        ret = webrtc::WebRtcAec_Process(aec, ptr_to_float_frame, 1, ptr_to_float_frame, WND_SIZE, 80, 0);
        //printf("WebRtcAec_Process:%d\n", ret);

        if (farcount++ < 80) {
            //continue;
        }

        GenerateShortFrame(ptr_to_float_frame, out_frame, WND_SIZE);
        write(fd_out, out_frame, frame_size);
    } while (true);

    close(fd_out);
    close(fd_near);
    close(fd_far);

    webrtc::WebRtcAec_Free(aec);
    printf("test aec end\n");
}

static void AGC(const char * infname, const char * ofname)
{
    short input[80], output[80];
    int ifd, ofd;
    void * agc = NULL;

    agc = WebRtcAgc_Create();
    WebRtcAgc_Init(agc, 0, 255, kAgcModeFixedDigital, 8000);

    WebRtcAgcConfig config;
    config.compressionGaindB = 20;
    config.limiterEnable = 1;
    config.targetLevelDbfs = 3;
    WebRtcAgc_set_config(agc, config);

    ifd = open(infname, O_RDONLY | O_BINARY);
    ofd = open(ofname, O_CREAT | O_WRONLY | O_TRUNC | O_BINARY, 0666);

    int nread = 0;
    int prevMicLevel = 30;

    while ((nread = read(ifd, input, sizeof(input))) > 0) {
        short * inarray[1] = {input};
        short * outarray[2] = { output };
        int outlevel = 0;
        uint8_t warning;

        int ret = WebRtcAgc_Process(agc, inarray, 1, 80, outarray,
            prevMicLevel, &outlevel, 0, &warning);
        if (ret != 0) {
            printf("WebRtcAgc_Process error:%d\n", ret);
            break;
        }
        //printf("mic level:%d, %d\n", prevMicLevel, outlevel);
        prevMicLevel = outlevel;
        write(ofd, output, nread);
    }

    close(ifd);
    close(ofd);

    delete agc;
}

static void NS(const char * infname, const char * ofname)
{
    short inpcm[80], outpcm[80];
    float inf[80], outf[80];
    float * inarray[1] = { inf };
    float * outarray[1] = { outf };
    int ifd, ofd, nread;

    NsHandle * ns = WebRtcNs_Create();
    WebRtcNs_Init(ns, 8000);
    WebRtcNs_set_policy(ns, 1);

    ifd = open(infname, O_RDONLY | O_BINARY);
    ofd = open(ofname, O_CREAT | O_WRONLY | O_TRUNC | O_BINARY, 0666);

    while ((nread = read(ifd, inpcm, sizeof(inpcm))) > 0) {
        GenerateFloatFrame(inpcm, 80, inarray);
        WebRtcNs_Analyze(ns, inarray[0]);
        WebRtcNs_Process(ns, inarray, 1, outarray);
        GenerateShortFrame(outarray, outpcm, 80);
        write(ofd, outpcm, nread);
    }

    close(ifd);
    close(ofd);

    WebRtcNs_Free(ns);
}

static void VAD(const char * infname)
{
    VadInst * vad = WebRtcVad_Create();
    WebRtcVad_Init(vad);
    WebRtcVad_set_mode(vad, 1);

    short inpcm[80];
    int fd = open(infname, O_RDONLY | O_BINARY);
    for (int i = 0; i < 20; i++) {
        if (read(fd, inpcm, sizeof(inpcm)) != sizeof(inpcm)) {
            break;
        }
        int ret = WebRtcVad_Process(vad, 8000, inpcm, 80);
        printf("WebRtcVad_Process:%d\n", ret);
    }

    close(fd);

    WebRtcVad_Free(vad);
}

int main(int argc, char * argv[])
{
    AEC();
    //AGC();
    //NS("../byby_8K_1C_16bit.pcm", "../byby_8K_1C_16bit_ns.pcm");
    //AGC("../out.pcm", "../out_agc.pcm");
    NS("../out.pcm", "../out_ns.pcm");
    VAD("../out_ns.pcm");

    printf("done\n");
    getchar();
    return 0;
}