#!/usr/bin/env python3
"""Adapt a Holosoma G1 WBT ONNX to MotionTrackingController without retraining.

Requires numpy and onnx. Deliberately accepts only the six standard, single-frame
proprioceptive WBT terms. The embedded experiment config is the source of truth.
"""
import argparse
import json
from pathlib import Path

import numpy as np
import onnx
from onnx import TensorProto, helper, numpy_helper
from onnx.reference import ReferenceEvaluator


def convert(source: Path, destination: Path):
    if source.resolve() == destination.resolve():
        raise ValueError("Write a separate adapted model; preserve the original.")
    original = onnx.load(source)
    if [x.name for x in original.graph.output] != [
        "actions", "joint_pos", "joint_vel", "ref_pos_xyz", "ref_quat_xyzw"
    ]:
        raise ValueError("Unexpected Holosoma WBT output signature.")
    model = onnx.ModelProto()
    model.CopyFrom(original)
    meta = {p.key: json.loads(p.value) for p in model.metadata_props}
    cfg = meta["experiment_config"]
    robot = cfg["robot"]
    group = cfg["observation"]["groups"]["actor_obs"]
    mapping = {
        "actions": "actions", "base_ang_vel": "base_ang_vel",
        "dof_pos": "joint_pos", "dof_vel": "joint_vel",
        "motion_command": "command", "motion_ref_ori_b": "motion_anchor_ori_b",
    }
    if set(group["terms"]) != set(mapping) or group["history_length"] != 1:
        raise ValueError("Only the standard six-term, history=1 WBT actor is supported.")
    for term in group["terms"].values():
        if term["scale"] != 1.0 or term.get("clip") is not None:
            raise ValueError("Per-term scaling/clipping requires a separate adapter.")
    names = meta["dof_names"]
    if len(names) != 29 or names != robot["dof_names"]:
        raise ValueError("Expected matching G1 29-DOF policy/config joint order.")
    motion = cfg["command"]["setup_terms"]["motion_command"]["params"]["motion_config"]
    anchors = motion["body_name_ref"]
    if len(anchors) != 1:
        raise ValueError("Expected exactly one reference body.")
    scales = meta.get("action_scale")
    if scales is None:
        control = robot["control"]
        scales = np.full(29, control["action_scale"], dtype=float)
        if control["action_scales_by_effort_limit_over_p_gain"]:
            scales *= np.array(robot["dof_effort_limit_list"]) / np.array(meta["kp"])
    scales = np.broadcast_to(np.asarray(scales), (29,))
    if not np.isfinite(scales).all():
        raise ValueError("Invalid action scales.")

    # ONNX stores only the reference torso. Publish it honestly as a one-body
    # motion (body_names=torso_link), rather than inventing other body states.
    prefix = "holosoma_adapter_"
    if any(any(x.startswith(prefix) for x in n.output) for n in model.graph.node):
        raise ValueError("Model is already adapted.")
    def const(name, value):
        full = prefix + name
        model.graph.initializer.append(numpy_helper.from_array(np.asarray(value), full))
        return full
    obs_clip = float(cfg["observation"]["clip_observations"])
    act_clip = float(robot["control"]["action_clip_value"])
    if not robot["control"]["clip_actions"]:
        raise ValueError("Expected clipped WBT actions.")
    lo = const("obs_lo", np.float32(-obs_clip))
    hi = const("obs_hi", np.float32(obs_clip))
    for node in model.graph.node:
        for i, name in enumerate(node.input):
            if name == "obs": node.input[i] = prefix + "obs"
            if name == "actions": node.input[i] = prefix + "raw_actions"
        for i, name in enumerate(node.output):
            if name == "actions": node.output[i] = prefix + "raw_actions"
    model.graph.node.insert(0, helper.make_node("Clip", ["obs", lo, hi], [prefix + "obs"]))
    model.graph.node.append(helper.make_node("Clip", [prefix + "raw_actions",
        const("act_lo", np.float32(-act_clip)), const("act_hi", np.float32(act_clip))], ["actions"]))
    axis = const("body_axis", np.array([1], np.int64))
    indices = const("wxyz", np.array([3, 0, 1, 2], np.int64))
    model.graph.node.extend([
        helper.make_node("Unsqueeze", ["ref_pos_xyz", axis], ["body_pos_w"]),
        helper.make_node("Gather", ["ref_quat_xyzw", indices], [prefix + "quat"], axis=1),
        helper.make_node("Unsqueeze", [prefix + "quat", axis], ["body_quat_w"]),
    ])
    model.graph.output.extend([
        helper.make_tensor_value_info("body_pos_w", TensorProto.FLOAT, [1, 1, 3]),
        helper.make_tensor_value_info("body_quat_w", TensorProto.FLOAT, [1, 1, 4]),
    ])
    values = {
        "joint_names": names, "joint_stiffness": meta["kp"], "joint_damping": meta["kd"],
        "default_joint_pos": [robot["init_state"]["default_joint_angles"][n] for n in names],
        "command_names": ["motion"],
        "observation_names": [mapping[n] for n in sorted(group["terms"])],
        "observation_history_lengths": [1] * len(mapping), "action_scale": scales.tolist(),
        "anchor_body_name": anchors[0], "body_names": anchors,
    }
    props = {p.key: p.value for p in model.metadata_props}
    props.update({k: ",".join(map(str, v)) if isinstance(v, list) else v for k, v in values.items()})
    helper.set_model_props(model, props)
    onnx.checker.check_model(model)

    # Exercise the original and adapted graphs with identical inputs, including
    # saturation, and check both action and reference output equivalence.
    before, after = ReferenceEvaluator(original), ReferenceEvaluator(model)
    rng = np.random.default_rng(42)
    for step, magnitude in [(0, 0), (1, 1), (100, 1), (1000, 200)]:
        obs = (rng.normal(size=(1, 154)) * magnitude).astype(np.float32)
        t = np.array([[step]], np.float32)
        # NumPy's ELU reference evaluates exp on the unused positive branch too.
        with np.errstate(over="ignore"):
            expected = before.run(None, {"obs": np.clip(obs, -obs_clip, obs_clip), "time_step": t})
            result = after.run(None, {"obs": obs, "time_step": t})
        if not all(np.isfinite(x).all() for x in expected + result):
            raise ValueError("Non-finite policy/reference output during validation.")
        if step == 0:
            initial_positions = dict(zip(names, map(float, expected[1][0])))
        got = dict(zip([x.name for x in model.graph.output], result))
        np.testing.assert_allclose(got["actions"], np.clip(expected[0], -act_clip, act_clip), atol=1e-5)
        np.testing.assert_allclose(got["joint_pos"], expected[1], atol=1e-6)
        np.testing.assert_allclose(got["joint_vel"], expected[2], atol=1e-6)
        np.testing.assert_allclose(got["body_pos_w"][:, 0], expected[3], atol=1e-6)
        np.testing.assert_allclose(got["body_quat_w"][:, 0], expected[4][:, [3, 0, 1, 2]], atol=1e-6)
    destination.parent.mkdir(parents=True, exist_ok=True)
    onnx.save(model, destination)
    destination.with_suffix('.initial_pose.json').write_text(json.dumps(initial_positions))
    print(f"Saved {destination}; graph and four numerical equivalence checks passed.")
    print("Controller observation order:", values["observation_names"])


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("source", type=Path)
    parser.add_argument("destination", type=Path)
    args = parser.parse_args()
    convert(args.source, args.destination)
