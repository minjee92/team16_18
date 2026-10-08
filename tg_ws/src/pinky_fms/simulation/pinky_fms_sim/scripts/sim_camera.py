#!/usr/bin/env python3
"""Set simulation camera resolution in generated URDF, preserving source models."""
import argparse
import sys
import xml.etree.ElementTree as ET


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('width', type=int)
    parser.add_argument('height', type=int)
    args = parser.parse_args()
    if not (160 <= args.width <= 3840 and 120 <= args.height <= 2160):
        parser.error('Camera resolution outside supported limits')
    root = ET.parse(sys.stdin).getroot()
    cameras = root.findall(".//sensor[@type='camera']/camera/image")
    if len(cameras) != 1:
        raise ValueError(f'Expected one camera, found {len(cameras)}')
    cameras[0].find('width').text = str(args.width)
    cameras[0].find('height').text = str(args.height)
    print(ET.tostring(root, encoding='unicode'))


if __name__ == '__main__':
    main()
