// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 The NOMAD Authors

using System;
namespace NOMAD.MissionPlanner
{
    internal class MavlinkFrameParser
    {
        public long ResyncCount;
        public long FramesEmitted;

        private const byte STX_V1 = 0xFE;
        private const byte STX_V2 = 0xFD;
        private const int MAX_FRAME = 280; // v2 + signature

        private readonly byte[] _buf = new byte[MAX_FRAME];
        private int _len;
        private int _expected; // expected total frame length, 0 = unknown yet

        public void Push(byte[] data, int length, Action<MavlinkFrame> onFrame)
        {
            for (int i = 0; i < length; i++)
            {
                byte b = data[i];

                if (_len == 0)
                {
                    if (b == STX_V1 || b == STX_V2)
                    {
                        _buf[_len++] = b;
                        _expected = 0;
                    }
                    continue;
                }

                if (_len < MAX_FRAME) _buf[_len++] = b;
                else { Resync(); continue; }

                if (_expected == 0 && _len >= 2)
                {
                    byte payloadLen = _buf[1];
                    if (_buf[0] == STX_V1) _expected = 6 + payloadLen + 2; // header(6)+payload+crc(2)
                    else if (_len >= 3)
                    {
                        // v2: header(10)+payload+crc(2)+signature(13 if MAVLINK_IFLAG_SIGNED bit 0
                        // of incompat_flags). incompat_flags is the third byte, so the frame length
                        // cannot be decided at _len == 2 — _buf[2] still holds stale data there.
                        bool signed = (_buf[2] & 0x01) != 0;
                        _expected = 10 + payloadLen + 2 + (signed ? 13 : 0);
                    }
                    if (_expected > MAX_FRAME) { Resync(); continue; }
                }

                if (_expected > 0 && _len >= _expected)
                {
                    EmitFrame(onFrame);
                    _len = 0; _expected = 0;
                }
            }
        }

        private void Resync()
        {
            ResyncCount++;
            // Find next STX in current buffer to recover (cheap heuristic)
            for (int j = 1; j < _len; j++)
            {
                if (_buf[j] == STX_V1 || _buf[j] == STX_V2)
                {
                    int rem = _len - j;
                    Buffer.BlockCopy(_buf, j, _buf, 0, rem);
                    _len = rem;
                    _expected = 0;
                    return;
                }
            }
            _len = 0; _expected = 0;
        }

        private void EmitFrame(Action<MavlinkFrame> onFrame)
        {
            FramesEmitted++;
            bool v2 = _buf[0] == STX_V2;
            byte payloadLen = _buf[1];

            byte seq, sysid, compid;
            uint msgid;
            int payloadOffset;

            if (v2)
            {
                seq = _buf[4];
                sysid = _buf[5];
                compid = _buf[6];
                msgid = (uint)(_buf[7] | (_buf[8] << 8) | (_buf[9] << 16));
                payloadOffset = 10;
            }
            else
            {
                seq = _buf[2];
                sysid = _buf[3];
                compid = _buf[4];
                msgid = _buf[5];
                payloadOffset = 6;
            }

            // We have to copy the raw frame because the parser reuses _buf.
            var raw = new byte[_expected];
            Buffer.BlockCopy(_buf, 0, raw, 0, _expected);

            onFrame(new MavlinkFrame
            {
                Raw = raw,
                RawLength = _expected,
                IsV2 = v2,
                Sysid = sysid,
                Compid = compid,
                Msgid = msgid,
                Seq = seq,
                PayloadOffset = payloadOffset,
                PayloadLength = payloadLen,
            });
        }
    }

    internal struct MavlinkFrame
    {
        public byte[] Raw;
        public int RawLength;
        public bool IsV2;
        public byte Sysid;
        public byte Compid;
        public uint Msgid;
        public byte Seq;
        public int PayloadOffset;
        public int PayloadLength;

        public bool IsHeartbeat => Msgid == 0; // MAVLINK_MSG_ID_HEARTBEAT
        public bool IsRadioStatus => Msgid == 109; // RADIO_STATUS
        public bool IsParamRequest => Msgid == 20 || Msgid == 21; // PARAM_REQUEST_READ / PARAM_REQUEST_LIST
        public bool IsParamValue => Msgid == 22; // PARAM_VALUE
    }
}
