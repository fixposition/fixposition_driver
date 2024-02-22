/**
 *  @file
 *  @brief Declaration of Fpb message framework
 *
 * \verbatim
 *  ___    ___
 *  \  \  /  /
 *   \  \/  /   Fixposition AG
 *   /  /\  \   All right reserved.
 *  /__/  \__\
 * \endverbatim
 *
 */

#ifndef __FIXPOSITION_DRIVER_LIB_FPB__
#define __FIXPOSITION_DRIVER_LIB_FPB__

#include <stdint.h>

namespace fixposition {

static uint32_t const k_crc32_fpb[] = {
    0x00000000, 0x32c00699, 0x65800d32, 0x57400bab, 0xcb001a64, 0xf9c01cfd,
    0xae801756, 0x9c4011cf, 0xa4c03251, 0x960034c8, 0xc1403f63, 0xf38039fa,
    0x6fc02835, 0x5d002eac, 0x0a402507, 0x3880239e, 0x7b40623b, 0x498064a2,
    0x1ec06f09, 0x2c006990, 0xb040785f, 0x82807ec6, 0xd5c0756d, 0xe70073f4,
    0xdf80506a, 0xed4056f3, 0xba005d58, 0x88c05bc1, 0x14804a0e, 0x26404c97,
    0x7100473c, 0x43c041a5, 0xf680c476, 0xc440c2ef, 0x9300c944, 0xa1c0cfdd,
    0x3d80de12, 0x0f40d88b, 0x5800d320, 0x6ac0d5b9, 0x5240f627, 0x6080f0be,
    0x37c0fb15, 0x0500fd8c, 0x9940ec43, 0xab80eada, 0xfcc0e171, 0xce00e7e8,
    0x8dc0a64d, 0xbf00a0d4, 0xe840ab7f, 0xda80ade6, 0x46c0bc29, 0x7400bab0,
    0x2340b11b, 0x1180b782, 0x2900941c, 0x1bc09285, 0x4c80992e, 0x7e409fb7,
    0xe2008e78, 0xd0c088e1, 0x8780834a, 0xb54085d3, 0xdfc18e75, 0xed0188ec,
    0xba418347, 0x888185de, 0x14c19411, 0x26019288, 0x71419923, 0x43819fba,
    0x7b01bc24, 0x49c1babd, 0x1e81b116, 0x2c41b78f, 0xb001a640, 0x82c1a0d9,
    0xd581ab72, 0xe741adeb, 0xa481ec4e, 0x9641ead7, 0xc101e17c, 0xf3c1e7e5,
    0x6f81f62a, 0x5d41f0b3, 0x0a01fb18, 0x38c1fd81, 0x0041de1f, 0x3281d886,
    0x65c1d32d, 0x5701d5b4, 0xcb41c47b, 0xf981c2e2, 0xaec1c949, 0x9c01cfd0,
    0x29414a03, 0x1b814c9a, 0x4cc14731, 0x7e0141a8, 0xe2415067, 0xd08156fe,
    0x87c15d55, 0xb5015bcc, 0x8d817852, 0xbf417ecb, 0xe8017560, 0xdac173f9,
    0x46816236, 0x744164af, 0x23016f04, 0x11c1699d, 0x52012838, 0x60c12ea1,
    0x3781250a, 0x05412393, 0x9901325c, 0xabc134c5, 0xfc813f6e, 0xce4139f7,
    0xf6c11a69, 0xc4011cf0, 0x9341175b, 0xa18111c2, 0x3dc1000d, 0x0f010694,
    0x58410d3f, 0x6a810ba6, 0x8d431a73, 0xbf831cea, 0xe8c31741, 0xda0311d8,
    0x46430017, 0x7483068e, 0x23c30d25, 0x11030bbc, 0x29832822, 0x1b432ebb,
    0x4c032510, 0x7ec32389, 0xe2833246, 0xd04334df, 0x87033f74, 0xb5c339ed,
    0xf6037848, 0xc4c37ed1, 0x9383757a, 0xa14373e3, 0x3d03622c, 0x0fc364b5,
    0x58836f1e, 0x6a436987, 0x52c34a19, 0x60034c80, 0x3743472b, 0x058341b2,
    0x99c3507d, 0xab0356e4, 0xfc435d4f, 0xce835bd6, 0x7bc3de05, 0x4903d89c,
    0x1e43d337, 0x2c83d5ae, 0xb0c3c461, 0x8203c2f8, 0xd543c953, 0xe783cfca,
    0xdf03ec54, 0xedc3eacd, 0xba83e166, 0x8843e7ff, 0x1403f630, 0x26c3f0a9,
    0x7183fb02, 0x4343fd9b, 0x0083bc3e, 0x3243baa7, 0x6503b10c, 0x57c3b795,
    0xcb83a65a, 0xf943a0c3, 0xae03ab68, 0x9cc3adf1, 0xa4438e6f, 0x968388f6,
    0xc1c3835d, 0xf30385c4, 0x6f43940b, 0x5d839292, 0x0ac39939, 0x38039fa0,
    0x52829406, 0x6042929f, 0x37029934, 0x05c29fad, 0x99828e62, 0xab4288fb,
    0xfc028350, 0xcec285c9, 0xf642a657, 0xc482a0ce, 0x93c2ab65, 0xa102adfc,
    0x3d42bc33, 0x0f82baaa, 0x58c2b101, 0x6a02b798, 0x29c2f63d, 0x1b02f0a4,
    0x4c42fb0f, 0x7e82fd96, 0xe2c2ec59, 0xd002eac0, 0x8742e16b, 0xb582e7f2,
    0x8d02c46c, 0xbfc2c2f5, 0xe882c95e, 0xda42cfc7, 0x4602de08, 0x74c2d891,
    0x2382d33a, 0x1142d5a3, 0xa4025070, 0x96c256e9, 0xc1825d42, 0xf3425bdb,
    0x6f024a14, 0x5dc24c8d, 0x0a824726, 0x384241bf, 0x00c26221, 0x320264b8,
    0x65426f13, 0x5782698a, 0xcbc27845, 0xf9027edc, 0xae427577, 0x9c8273ee,
    0xdf42324b, 0xed8234d2, 0xbac23f79, 0x880239e0, 0x1442282f, 0x26822eb6,
    0x71c2251d, 0x43022384, 0x7b82001a, 0x49420683, 0x1e020d28, 0x2cc20bb1,
    0xb0821a7e, 0x82421ce7, 0xd502174c, 0xe7c211d5
};

uint32_t Crc32fpb(const uint8_t* data, const int size) {
    uint32_t crc = 0;
    if (data != nullptr) {
        for (int ix = 0; ix < size; ix++) {
            crc = (crc << 8) ^ k_crc32_fpb[((crc >> 24) ^ data[ix]) & 0xff];
        }
    }
    return crc;
}

struct FpbHeader {
    uint8_t  sync1;         //!< FP_B frame sync char 1 (0x66)
    uint8_t  sync2;         //!< FP_B frame sync char 2 (0x21)
    uint16_t msg_id;        //!< ID of the FP_B message (2001 for measurements message)
    uint16_t payload_size;  //!< Size of the payload
    uint16_t time;          //!< Time of the message. Unused, set to 0.
};
const int FP_B_HEAD_SIZE = 8;  //!< Size of FP_B frame header
const int FP_B_CRC_SIZE = 4;   //!< Size of FP_B crc

}  // namespace fixposition

#endif  // __FIXPOSITION_DRIVER_LIB_FPB__
