#!/usr/bin/env python3
"""Deterministic failure-injection test for ordinal OV5647 YUV/metadata pairing.

Simulates independent frame/metadata drops. This is NOT a hardware test and
does not prove libcamera frame identity. Does not touch FC or WORKED5.
"""
import argparse

def fifo_pair(frames, metadata):
    return list(zip(frames, metadata))

def test_case(name, frames, metadata):
    pairs=fifo_pair(frames,metadata)
    wrong=[(f,m) for f,m in pairs if f!=m]
    first=wrong[0] if wrong else None
    print(f"INJECT case={name} frames={len(frames)} metadata={len(metadata)} "
          f"pairs={len(pairs)} incorrect={len(wrong)} "
          f"first_incorrect={first} "
          f"tail_offset={(pairs[-1][0]-pairs[-1][1]) if pairs else 'NA'}")
    return len(wrong)

def main():
    p=argparse.ArgumentParser()
    p.add_argument("--count",type=int,default=300)
    p.add_argument("--drop-at",type=int,default=100)
    args=p.parse_args()
    if args.count<4 or not 1<=args.drop_at<args.count-1:
        p.error("need count>=4 and 1<=drop-at<count-1")
    ids=list(range(args.count))
    normal=test_case("normal",ids,ids)
    missing_frame=test_case("drop_yuv_frame",
                            [i for i in ids if i!=args.drop_at],ids)
    missing_metadata=test_case("drop_metadata",ids,
                               [i for i in ids if i!=args.drop_at])
    # A gap in sensor timestamps could reveal a metadata loss, but cannot
    # detect a dropped video frame when the raw stream has no frame IDs.
    print("CONCLUSION ordinal_FIFO_cannot_recover_identity_after_independent_drop")
    print("NOTE synthetic fault injection; not a real camera drop observation")
    return 0 if normal==0 and missing_frame>0 and missing_metadata>0 else 1

if __name__=="__main__":
    raise SystemExit(main())
