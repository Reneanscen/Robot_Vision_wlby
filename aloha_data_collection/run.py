#!/usr/bin/env python
# -*- coding: UTF-8 -*-
"""
@Project : aloha_data_collection
@File    : run.py
@IDE     : PyCharm
@Author  : bhuang
@Date    : 2024/4/9 17:13:15
"""

import os
import rclpy
import yaml
from argparse import ArgumentParser

from data_record import DataRecordNode


def parse_config_yaml(config_path):
    return yaml.safe_load(open(config_path, 'r'))


def main(configs):
    record_config = configs["record_node"]
    record_config.update(configs["cameras"])

    rclpy.init()
    node = DataRecordNode(record_config)
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    parser = ArgumentParser(description="aloha data collection")
    parser.add_argument("--config", type=str, default="./configs/right_arm_data_record.yaml")
    args = parser.parse_args()

    assert os.path.exists(args.config)

    cfg = parse_config_yaml(args.config)
    main(cfg)
