import json

import maya.cmds as cmds
import pymel.core as pm


EXPORT_NODE_NAME = "mgearFbxExportNode"


class FbxExportNode(object):
    VERSION = 0
    TYPE_ATTR = "fbxExportNode"
    EXPORT_DATA_ATTR = "exportData"
    EXPORT_DATA = {
        "geo_root": "",
        "joint_root": "",
        "version": VERSION,
        "up_axis": "Y",
        "file_type": "Binary",
        "fbx_version": "FBX 2020",
        "remove_namespace": True,
        "scene_clean": True,
        "use_partitions": True,
        "file_name": "",
        "file_path": "",
        "skinning": True,
        "blendshapes": True,
        "deformations": True,
        "partitions": dict(),
        "animClips": dict(),
    }
    ANIM_CLIP_DATA = {
        "title": "Untitled",
        "path": "",
        "enabled": True,
        "startOrigin": True,
        "frameZero": False,
        "frameRate": "30 FPS",
        "startFrame": int(pm.playbackOptions(min=True, query=True)),
        "endFrame": int(pm.playbackOptions(max=True, query=True)),
    }

    def __init__(self, node=None):
        self._node = node
        self._export_data = dict()  # internal export data
        data = self._parse_export_data()
        if data is None:
            data = self.EXPORT_DATA
        self._save_data(data)

    @property
    def export_data(self):
        return self._export_data

    @export_data.setter
    def export_data(self, value):
        self._export_data = value

    @classmethod
    def create(cls, name=EXPORT_NODE_NAME):
        return cls(cmds.createNode("network", name=name))

    @classmethod
    def create_from_export_data(cls, data):
        export_node = cls()
        export_node.data = data
        return export_node

    @classmethod
    def get(cls, name=EXPORT_NODE_NAME):
        if not cls.exists_in_scene(name):
            return None
        return cls(name)

    @classmethod
    def find(cls):
        found_export_nodes = list()
        network_nodes = cmds.ls(type="network")
        for network_node in network_nodes:
            if not cmds.attributeQuery(cls.TYPE_ATTR, node=network_node, exists=True):
                continue
            found_export_nodes.append(cls(network_node))

        return found_export_nodes

    @classmethod
    def exists_in_scene(cls, name=EXPORT_NODE_NAME):
        if not name or not cmds.objExists(name):
            return False
        if not cmds.attributeQuery(cls.TYPE_ATTR, node=name, exists=True):
            return False
        return True

    def save_root_data(self, root_type, root_names):
        # root type = geo_root or joint_root
        data = self._parse_export_data()
        data[root_type] = root_names
        return self._save_data(data)

    def add_new_skeletal_mesh_partition(self, name, node_names):
        data = self._parse_export_data()
        data.setdefault("partitions", dict())
        if name in data["partitions"]:
            cmds.warning(
                'Partition with name "{}" already exist!'.format(name)
            )
            return False

        data["partitions"][name] = {
            "enabled": True,
            "color": [241, 90, 91],
            "skeletalMeshes": node_names,
        }

        return self._save_data(data)

    def delete_skeletal_mesh_partition(self, name):
        data = self._parse_export_data()
        partitions = data.get("partitions", dict())
        if not partitions or name not in partitions:
            return False
        partitions.pop(name)

        return self._save_data(data)

    def get_partitions(self):
        return self._parse_export_data().get("partitions", dict())

    def set_partition_enabled(self, name, flag):
        data = self._parse_export_data()
        partitions = data.get("partitions", dict())
        if not partitions or name not in partitions:
            return False
        partitions[name]["enabled"] = flag

        return self._save_data(data)

    def set_partition_name(self, name, new_name):
        data = self._parse_export_data()
        partitions = data.get("partitions", dict())
        if not partitions or name not in partitions:
            return False

        partitions[new_name] = partitions.pop(name)

        return self._save_data(data)

    def set_partition_color(self, name, new_color):
        data = self._parse_export_data()
        partitions = data.get("partitions", dict())
        if not partitions or name not in partitions:
            return False

        partitions[name]["color"] = new_color

        return self._save_data(data)

    def add_skeletal_meshes_to_partition(self, name, skeletal_meshes):
        data = self._parse_export_data()
        partitions = data.get("partitions", dict())
        if not partitions or name not in partitions:
            return False

        current_skeletal_meshes = partitions.get(name, dict()).get(
            "skeletalMeshes", list()
        )
        valid_skeletal_meshes = [
            skeletal_mesh
            for skeletal_mesh in skeletal_meshes
            if skeletal_mesh not in current_skeletal_meshes
        ]
        if not valid_skeletal_meshes:
            return

        partitions[name].setdefault("skeletalMeshes", list())
        partitions[name]["skeletalMeshes"].extend(valid_skeletal_meshes)

        return self._save_data(data)

    def delete_skeletal_mesh_from_partition(
        self, name, skeletal_mesh_to_remove
    ):
        data = self._parse_export_data()
        partitions = data.get("partitions", dict())
        if not partitions or name not in partitions:
            return False

        skeletal_meshes = partitions[name].get("skeletalMeshes", list())
        if skeletal_mesh_to_remove not in skeletal_meshes:
            return False

        skeletal_meshes.remove(skeletal_mesh_to_remove)

        return self._save_data(data)

    def get_animation_clips(self, root_joint_name):
        return (
            self._parse_export_data()
            .get("animClips", dict())
            .get(root_joint_name, list())
        )

    def get_animation_clip(self, root_joint_name, clip_name):
        anim_clips = self.get_animation_clips(root_joint_name)
        if not anim_clips:
            return dict()

        found_anim_clip = dict()
        for anim_clip in anim_clips:
            anim_clip_title = anim_clip.get("title", None)
            if not anim_clip_title or anim_clip_title != clip_name:
                continue
            found_anim_clip = anim_clip
            break

        return found_anim_clip

    def add_animation_clip(self, root_joint_name, anim_clip_data=None):
        sequences = self.get_animation_clips(root_joint_name)
        total_sequences = len(sequences)
        clip_data = FbxExportNode.ANIM_CLIP_DATA.copy()
        clip_data["title"] = "Clip {}".format(total_sequences + 1)
        clip_data.update(
            anim_clip_data if anim_clip_data is not None else dict()
        )

        data = self._parse_export_data()
        anim_clips = data.get("animClips", dict()).get(root_joint_name, list())
        anim_clips.append(clip_data)

        data.setdefault("animClips", dict())
        data["animClips"].setdefault(root_joint_name, list())
        data["animClips"][root_joint_name] = anim_clips

        self._save_data(data)

        return clip_data["title"]

    def update_animation_clip(self, root_joint_name, clip_name, clip_data):
        data = self._parse_export_data()
        anim_clips = data.get("animClips", dict()).get(root_joint_name, list())
        index_to_update = None
        for i, anim_clip in enumerate(anim_clips):
            anim_clip_name = anim_clip.get("title", None)
            if not anim_clip_name or anim_clip_name != clip_name:
                continue
            index_to_update = i
            break
        if index_to_update is None:
            return
        anim_clips[index_to_update] = clip_data

        return self._save_data(data)

    def delete_animation_clip(self, root_joint_name, clip_name):
        data = self._parse_export_data()
        anim_clips = data.get("animClips", dict()).get(root_joint_name, list())
        index_to_remove = None
        for i, anim_clip in enumerate(anim_clips):
            anim_clip_name = anim_clip.get("title", None)
            if not anim_clip_name or anim_clip_name != clip_name:
                continue
            index_to_remove = i
            break
        if index_to_remove is None:
            return False
        anim_clips.pop(index_to_remove)

        return self._save_data(data)

    def delete_all_animation_clips(self):
        data = self._parse_export_data()
        data["animClips"] = dict()

        return self._save_data(data)

    def _get_attr_namespace(self, namespace, attr):
        return "{}.{}".format(namespace, attr)

    def _add_attr(self, node, attr, value):
        # create attribute
        if not cmds.attributeQuery(attr, node=node, exists=True):
            if isinstance(value, (str, list, dict)):
                cmds.addAttr(node, longName=attr, dataType="string")
            else:
                attr_type = type(value).__name__
                if attr_type == "int":
                    attr_type = "long"
                cmds.addAttr(node, longName=attr, attributeType=attr_type)

        # set attribute value
        attr_namespace = self._get_attr_namespace(node, attr)
        if isinstance(value, (str, list, dict)):
            cmds.setAttr(attr_namespace, value, type="string")
        else:
            cmds.setAttr(attr_namespace, value)

    def _save_data(self, data):
        if not self._node or not cmds.objExists(self._node):
            return False

        self._add_attr(self._node, self.TYPE_ATTR, True)
        for attr, value in data.items():
            self._add_attr(self._node, attr, value)

        try:
            json_data = json.dumps(data)
        except Exception as exc:
            cmds.warning(
                "Error while converting FbxExportNode data into a dictionary."
            )
            return False
        self._add_attr(self._node, self.EXPORT_DATA_ATTR, json_data)
        self._export_data = data
        return True

    def _parse_export_data(self):
        if not (cmds.objExists(self._node) and
                cmds.attributeQuery(self.EXPORT_DATA_ATTR,
                                    node=self._node,
                                    exists=True)):
            return None
        export_data_str = self._get_attr_namespace(self._node,
                                                   self.EXPORT_DATA_ATTR)
        export_data = cmds.getAttr(export_data_str)
        try:
            self._export_data = json.loads(export_data)
        except Exception:
            cmds.warning("Error while parsing FbxExportNode export data.")
        return self._export_data
