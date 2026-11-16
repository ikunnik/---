"""DDS 环境自动修正（从 main.py 拆出）。"""
import logging
import os


def _dds_uri_has_topic_discovery(uri: str) -> bool:
    """URI 可能是内联 XML，也可能指向文件；两种都看内容。"""
    if not uri:
        return False
    if uri.lstrip().startswith("<"):
        return "EnableTopicDiscoveryEndpoints" in uri
    path = os.path.expanduser(uri[7:] if uri.startswith("file://") else uri)
    try:
        with open(path, "r", encoding="utf-8") as file:
            return "EnableTopicDiscoveryEndpoints" in file.read()
    except OSError:
        return False


def _ensure_dds_config(cfg):
    """确保 CYCLONEDDS_URI 带主题发现，否则后启动的相机 reader 可能收不到帧。"""
    uri = os.environ.get("CYCLONEDDS_URI", "")
    if _dds_uri_has_topic_discovery(uri):
        return

    path = os.path.expanduser(cfg.dds_uri_file)
    if os.path.isfile(path):
        os.environ["CYCLONEDDS_URI"] = "file://" + path
        if uri:
            logging.warning(
                "[主进程] CYCLONEDDS_URI 未包含主题发现（或文件不可读），已改用 %s",
                path,
            )
        else:
            logging.info("[主进程] 未设置 CYCLONEDDS_URI，使用 %s", path)
    elif not uri:
        logging.warning(
            "[主进程] 未设置 CYCLONEDDS_URI，且找不到 %s；DDS 可能收不到相机数据",
            path,
        )
    else:
        logging.warning(
            "[主进程] CYCLONEDDS_URI 未包含 EnableTopicDiscoveryEndpoints；"
            "相机可能收不到数据"
        )



ensure = _ensure_dds_config
