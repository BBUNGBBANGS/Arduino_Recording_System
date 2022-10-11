
#define WAVFILE_HEADER_LENGTH   (44)
#define FMT_SIZE            (16)
#define AUDIO_FORMAT        (1)
#define NUMBER_OF_CHANNEL   (1)
#define SAMPLE_RATE         (45454)
#define BPS                 (16)
#define BLOCK_ALIGN         (NUMBER_OF_CHANNEL * (BPS/8))
#define BYTE_RATE           (SAMPLE_RATE * NUMBER_OF_CHANNEL * (BPS/8))

typedef struct
{
    uint8_t ChunkID[4];       // Contains the letters "RIFF" in ASCII form
    uint32_t ChunkSize;       // This is the size of the rest of the chunk following this number
    uint8_t Format[4];        // Contains the letters "WAVE" in ASCII form
}Riff_t;
//-------------------------------------------
// [Channel]
// - streo     : [left][right]
// - 3 channel : [left][right][center]
// - quad      : [front left][front right][rear left][reat right]
// - 4 channel : [left][center][right][surround]
// - 6 channel : [left center][left][center][right center][right][surround]
//-------------------------------------------
typedef struct
{
    uint8_t ChunkID[4];       // Contains the letters "fmt " in ASCII form
    uint32_t ChunkSize;       // 16 for PCM.  This is the size of the rest of the Subchunk which follows this number.
    uint16_t AudioFormat;     // PCM = 1
    uint16_t NumChannels;     // Mono = 1, Stereo = 2, etc.
    uint32_t SampleRate;      // 8000, 44100, etc.
    uint32_t ByteRate;        // SampleRate * NumChannels * BitsPerSample/8
    uint16_t BlockAlign;      // NumChannels * BitsPerSample/8
    uint16_t BitPerSample;    // 8 bits = 8, 16 bits = 16, etc
}Fmt_t;

typedef struct
{
    uint8_t ChunkID[4];       // Contains the letters "data" in ASCII form
    uint32_t ChunkSize;       // NumSamples * NumChannels * BitsPerSample/8
}Data_t;

typedef struct
{
    Riff_t Riff;
    Fmt_t Fmt;
    Data_t Data;
}Wave_Header_t;
