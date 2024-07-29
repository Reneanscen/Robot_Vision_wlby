import os
import sys
import oss2
from enum import IntEnum

from core.logger import logger
from configs import MODEL_SAVE_DIR, TASK_MAP


__all__ = ["check_model"]


class UploadStatus(IntEnum):
    success = 0
    failed = 1
    exist = 2


class DownloadStatus(IntEnum):
    success = 0
    failed = 1


def percentage(consumed_bytes, total_bytes):
    if total_bytes:
        rate = (float(consumed_bytes) / float(total_bytes))
        total_length = 50
        progress = int(total_length * rate)
        spaces = total_length - progress
        print(f'downloading: {"#" * progress}{" " * spaces}|{int( 100 *rate)}% ', end='\r')
        sys.stdout.flush()


class ZmOSS(object):

    def __init__(self, access_key='', secrets='', bucket_name=''):
        auth = oss2.Auth(access_key, secrets)
        self.bucket = oss2.Bucket(auth, 'http://oss-cn-hangzhou.aliyuncs.com', bucket_name)     # noqa

        self.headers = {
            'User-Agent': 'Mozilla/5.0 (Windows NT 10.0; WOW64) AppleWebKit/537.36 (KHTML, like Gecko) Chrome/55.0.2883.87 Safari/537.36',
            'Referer': 'http://music.163.com/song?id=4466775',      # noqa
            # 'Content-Type': 'application/octet-stream'
        }

        # local save directory
        self.save_dir = MODEL_SAVE_DIR
        if not os.path.exists(self.save_dir):
            os.makedirs(self.save_dir)

    def upload_file(self, image_obj, save_path, image_name):
        oss_path = save_path + '/' + image_name
        return self.up_file(oss_path, image_obj)

    def download_file_to_obj(self, file_path=None, prefix="wlby_models"):
        """
        return True if download success
        """
        if file_path:
            save_dir = os.path.join(self.save_dir, prefix, os.path.dirname(file_path))
            if not os.path.exists(save_dir):
                os.makedirs(save_dir)
            save_file = os.path.join(self.save_dir, prefix, file_path)
            file_path = os.path.join(prefix, file_path)

            try:
                self.bucket.get_object_to_file(file_path, save_file, progress_callback=percentage)
                print()
                return True
            except IOError:
                logger.info('download failed: {}'.format(file_path))
                return False

    def up_file(self, oss_path, file_obj):
        # 先检测oss上是否有该文件
        exist = self.bucket.object_exists(oss_path)
        if exist:
            return UploadStatus.exist.value
        else:
            logger.info('start upload')
            # 上传文件
            print(oss_path, file_obj)
            result1 = self.bucket.put_object(oss_path, file_obj)
            if int(result1.status) != 200:
                logger.info('oss upload is failed: %s' % oss_path)
                return UploadStatus.failed.value
            logger.info('upload succeed: %s' % oss_path)
            return UploadStatus.success.value


def main():
    access_key = "LTAI5t5zwLWyJgTsBob1mi4y"
    secrets = "SvxuDevGtbUwnILKqFsUDsHSZnaEUV"
    bucket_name = "wlby-robot"
    fabric_oss = ZmOSS(access_key, secrets, bucket_name)
    return fabric_oss


oss_handler = main()
upload_handler = oss_handler.upload_file
download_handler = oss_handler.download_file_to_obj


def check_model(prefix="wlby_models"):
    for task_id, task in TASK_MAP.items():
        model_files_key = [key for key in task[1].keys() if "model_path" in key]
        for model_file_key in model_files_key:
            model_file = task[1][model_file_key]
            if not os.path.exists(os.path.join(MODEL_SAVE_DIR, prefix, model_file)) and model_file:
                logger.info(f"starting download model {model_file}: ")
                download_handler(file_path=model_file, prefix=prefix)

    logger.info("all model check pass")


if __name__ == '__main__':
    ret = download_handler(file_path="wlby_models/bottle_v1_sym_12_140.pth")

