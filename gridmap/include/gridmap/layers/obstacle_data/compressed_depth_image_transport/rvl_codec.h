// IMPORTANT: This was copied from upstream image_transport_plugins to be able to use the decode function directly
// without needing a modified version of the package

#ifndef COMPRESSED_DEPTH_IMAGE_TRANSPORT_RVL_CODEC_H_
#define COMPRESSED_DEPTH_IMAGE_TRANSPORT_RVL_CODEC_H_

namespace gridmap
{

namespace compressed_depth_image_transport
{

class RvlCodec
{
  public:
    RvlCodec();
    // Compress input data into output. The size of output can be equal to
    // (1.5 * numPixels + 4) in the worst case.
    int CompressRVL(const unsigned short* input, unsigned char* output, int numPixels);
    // Decompress input data into output. The size of output must be
    // equal to numPixels.
    void DecompressRVL(const unsigned char* input, unsigned short* output, int numPixels);

  private:
    RvlCodec(const RvlCodec&);
    RvlCodec& operator=(const RvlCodec&);

    void EncodeVLE(int value);
    int DecodeVLE();

    int* buffer_;
    int* pBuffer_;
    int word_;
    int nibblesWritten_;
};

}  // namespace compressed_depth_image_transport

}  // namespace gridmap

#endif  // COMPRESSED_DEPTH_IMAGE_TRANSPORT_RVL_CODEC_H_
