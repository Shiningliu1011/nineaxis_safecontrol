import multiprocessing
import mmap
import os
import sys
import threading
import time
import uuid
from pathlib import Path

import jax
import numpy as np
import pytest
import trimesh

from collision_scene_inputs import scene_input, signed_scene
from test_collision_safety import _config, _identities
from work.collision_safety import CollisionSafety, CollisionStatus
from work.fixed_environment import FixedEnvironmentModel
from work.lidar_occupancy import LidarOccupancyScene, OccupancyGrid
from work.shared_collision_scene import ScenePublicationError, SharedSceneReader, SharedSceneWriter


@pytest.fixture
def shared_path():
    path = Path("/dev/shm") / ("robot-scene-test-" + uuid.uuid4().hex)
    yield path
    path.unlink(missing_ok=True)
    Path(str(path) + ".writer.lock").unlink(missing_ok=True)


def fixed_scene(safety, identities):
    grid = OccupancyGrid(np.full(3, .0015), (1, 1, 1), 1.)
    identities = identities._replace(required_space_hash=np.frombuffer(grid.required_space_hash, "u1"))
    producer = LidarOccupancyScene(safety.config, identities, grid)
    mesh = trimesh.creation.box(extents=[.004, .004, .004])
    mesh.apply_translation([.002, .002, .002])
    model = FixedEnvironmentModel.from_mesh(
        mesh, policy=safety.config.policy, model_revision=1, source_hash=b"f" * 32,
        validation_evidence="analytic closed cube in base_link, edge 4 mm", error_mm=0.)
    producer.set_fixed_environment(model)
    return producer.snapshot(now_ns=time.monotonic_ns()).scene


def run_writer(path, connection):
    safety = CollisionSafety(_config())
    with SharedSceneWriter(path, safety) as writer:
        connection.send(writer.epoch)
        while True:
            command = connection.recv()
            if command == "close":
                break
            if command == "exit":
                os._exit(17)
            if command == "publish":
                notice = writer.publish(scene_input(safety.config, _identities(safety.config)))
                connection.send(notice)
            if command == "burst":
                connection.send("started")
                for _ in range(300):
                    writer.publish(scene_input(safety.config, _identities(safety.config)))
                connection.send("finished")


def run_reader_constructor(path, connection):
    safety = CollisionSafety(_config())
    connection.send("entering_reader_constructor")
    try:
        with SharedSceneReader(path, safety, _identities(safety.config), slot_timeout_ns=1_000_000_000):
            connection.send("opened")
    except ValueError as error:
        connection.send((type(error).__name__, str(error)))


def test_reader_rejects_a_fifo_without_waiting_for_another_endpoint(shared_path):
    os.mkfifo(shared_path, mode=0o600)
    context = multiprocessing.get_context("spawn")
    parent, child = context.Pipe()
    process = context.Process(target=run_reader_constructor, args=(str(shared_path), child))
    process.start()
    try:
        assert parent.poll(20)
        assert parent.recv() == "entering_reader_constructor"
        assert parent.poll(2), "FIFO reader 构造仍然等待另一个端点"
        kind, reason = parent.recv()
        assert kind == "ValueError"
        assert "普通文件" in reason
        process.join(10)
        assert process.exitcode == 0
    finally:
        if process.is_alive():
            process.terminate()
            process.join(10)
        parent.close()
        child.close()


def test_independent_writer_publishes_readonly_scene_once_per_revision(shared_path):
    path = shared_path
    context = multiprocessing.get_context("spawn")
    parent, child = context.Pipe()
    process = context.Process(target=run_writer, args=(str(path), child))
    process.start()
    try:
        assert parent.poll(20)
        epoch = parent.recv()
        parent.send("publish")
        assert parent.poll(20)
        notice = parent.recv()
        safety = CollisionSafety(_config())
        with SharedSceneReader(path, safety, _identities(safety.config), slot_timeout_ns=10_000_000_000) as reader:
            assert reader.readonly
            result = reader.refresh()
            assert result.status == CollisionStatus.OK
            assert result.revision == notice
            assert bytes(np.asarray(result.prepared_scene.identities.scene_epoch)) == epoch
            assert result.prepared_scene.support_points_m.dtype == np.float64
            assert isinstance(result.prepared_scene.support_points_m, jax.Array)
            np.testing.assert_allclose(result.prepared_scene.support_points_m, [[.1, .2, .3], [.4, .5, .6], [0., 0., 0.]])
            assert reader.refresh().prepared_scene is result.prepared_scene
            assert reader.device_transfer_count == 1
            assert reader.prepared_scene is result.prepared_scene
        parent.send("close")
        process.join(10)
        assert process.exitcode == 0
    finally:
        if process.is_alive():
            process.terminate()
            process.join(10)
        parent.close()
        child.close()


def test_fixed_environment_and_complete_time_model_survive_transport(shared_path):
    safety = CollisionSafety(_config())
    with SharedSceneWriter(shared_path, safety) as writer:
        scene = fixed_scene(safety, _identities(safety.config))
        writer.publish(scene)
        with SharedSceneReader(shared_path, safety, scene.identities, slot_timeout_ns=10_000_000_000) as reader:
            result = reader.refresh()
            assert result.status == CollisionStatus.OK
            prepared = result.prepared_scene
            fixed = prepared.fixed_environment_model
            assert fixed.model_hash == scene.fixed_environment_model.model_hash
            assert fixed.source_hash == b"f" * 32
            assert fixed.validation_evidence == "analytic closed cube in base_link, edge 4 mm"
            np.testing.assert_allclose(fixed.support_points_m, [[.002, .002, .002]])
            assert fixed.support_demand == 1
            assert prepared.support_time_model is not None
            np.testing.assert_array_equal(prepared.support_time_model.source_history_hash,
                                          scene.support_time_model.source_history_hash)
            assert int(prepared.support_time_model.reference_stamp_ns) == int(scene.prepared_stamp_ns)


def test_writer_refuses_to_replace_an_unrelated_existing_file(shared_path):
    original = b"independent source evidence; preserve these bytes"
    shared_path.write_bytes(original)
    with pytest.raises(ValueError, match="共享场景"):
        SharedSceneWriter(shared_path, CollisionSafety(_config()))
    assert shared_path.read_bytes() == original


def test_writer_rejects_scene_preparation_from_before_its_epoch(shared_path):
    safety = CollisionSafety(_config())
    scene = scene_input(safety.config, _identities(safety.config))
    with SharedSceneWriter(shared_path, safety) as writer:
        with pytest.raises(ScenePublicationError, match="epoch"):
            writer.publish(scene)
        assert writer.revision.status == CollisionStatus.INVALID_SCENE


def test_invalid_time_model_source_revokes_cached_scene_before_publish_failure(shared_path):
    safety = CollisionSafety(_config())
    with SharedSceneWriter(shared_path, safety) as writer:
        scene = fixed_scene(safety, _identities(safety.config))
        writer.publish(scene)
        with SharedSceneReader(shared_path, safety, scene.identities, slot_timeout_ns=10_000_000_000) as reader:
            assert reader.refresh().status == CollisionStatus.OK
            corrupted = signed_scene(scene._replace(support_time_model=scene.support_time_model._replace(
                scene_epoch=np.zeros(16, dtype=np.uint8))))
            with pytest.raises(ScenePublicationError, match="INVALID_SCENE"):
                writer.publish(corrupted)
            assert reader.refresh().status == CollisionStatus.INVALID_SCENE
            assert reader.prepared_scene is None


def test_only_one_writer_and_revision_jumps_require_explicit_fault_reset(shared_path):
    safety = CollisionSafety(_config())
    identities = _identities(safety.config)
    with SharedSceneWriter(shared_path, safety) as writer:
        with pytest.raises(RuntimeError, match="writer"):
            SharedSceneWriter(shared_path, safety)
        writer.publish(scene_input(safety.config, identities))
        with SharedSceneReader(shared_path, safety, identities, slot_timeout_ns=10_000_000_000) as reader:
            assert reader.refresh().status == CollisionStatus.OK
            writer.publish(scene_input(safety.config, identities))
            writer.publish(scene_input(safety.config, identities))
            result = reader.refresh()
            assert result.status == CollisionStatus.INVALID_SCENE
            assert result.reason == "revision_jump"
            assert reader.prepared_scene is None
            assert reader.device_transfer_count == 1
            assert reader.refresh().status == CollisionStatus.INVALID_SCENE
            reader.reset_fault()
            assert reader.refresh().status == CollisionStatus.OK
            assert reader.device_transfer_count == 2


def test_writer_close_revokes_cache_and_new_epoch_needs_acceptance(shared_path):
    safety = CollisionSafety(_config())
    identities = _identities(safety.config)
    with SharedSceneWriter(shared_path, safety) as writer:
        writer.publish(scene_input(safety.config, identities))
        reader = SharedSceneReader(shared_path, safety, identities, slot_timeout_ns=10_000_000_000)
        assert reader.refresh().status == CollisionStatus.OK
    try:
        assert reader.refresh().status == CollisionStatus.INVALID_SCENE
        assert reader.prepared_scene is None
        with SharedSceneWriter(shared_path, safety) as replacement:
            replacement.publish(scene_input(safety.config, identities))
            assert reader.refresh().status == CollisionStatus.REVISION_PENDING
            assert reader.prepared_scene is None
            reader.accept_epoch(replacement.epoch)
            assert reader.refresh().status == CollisionStatus.OK
    finally:
        reader.close()


@pytest.mark.parametrize("corruption", ["slot", "header"])
def test_real_shared_memory_corruption_revokes_cached_device_scene(shared_path, corruption):
    safety = CollisionSafety(_config())
    identities = _identities(safety.config)
    with SharedSceneWriter(shared_path, safety) as writer:
        notice = writer.publish(scene_input(safety.config, identities))
        with SharedSceneReader(shared_path, safety, identities, slot_timeout_ns=10_000_000_000) as reader:
            assert reader.refresh().status == CollisionStatus.OK
            with shared_path.open("r+b") as source, mmap.mmap(source.fileno(), 0) as memory:
                offset = 8 if corruption == "header" else writer.layout.slot_offset(notice.active_slot) + 32
                memory[offset] ^= 1
            assert reader.refresh().status == CollisionStatus.INVALID_SCENE
            assert reader.prepared_scene is None
            assert reader.device_transfer_count == 1


def test_slot_timeout_revokes_local_reference_without_another_device_transfer(shared_path):
    safety = CollisionSafety(_config())
    identities = _identities(safety.config)
    with SharedSceneWriter(shared_path, safety) as writer:
        writer.publish(scene_input(safety.config, identities))
        with SharedSceneReader(shared_path, safety, identities, slot_timeout_ns=100_000_000) as reader:
            assert reader.refresh().status == CollisionStatus.OK
            time.sleep(.12)
            result = reader.refresh()
            assert result.status == CollisionStatus.INVALID_SCENE
            assert result.reason == "slot_timeout"
            assert reader.prepared_scene is None
            assert reader.device_transfer_count == 1


@pytest.mark.parametrize("invalidate_during_prepare", [False, True])
def test_new_revision_removes_the_public_scene_before_real_device_preparation(shared_path, invalidate_during_prepare):
    safety = CollisionSafety(_config())
    identities = _identities(safety.config)
    entered, release = threading.Event(), threading.Event()
    outcome = []
    with SharedSceneWriter(shared_path, safety) as writer:
        writer.publish(scene_input(safety.config, identities))
        with SharedSceneReader(shared_path, safety, identities, slot_timeout_ns=10_000_000_000) as reader:
            assert reader.refresh().status == CollisionStatus.OK
            notice = writer.publish(scene_input(safety.config, identities))

            def observe_call(frame, event, argument):
                if event == "call" and frame.f_code is CollisionSafety.prepare_scene.__code__:
                    entered.set()
                    assert release.wait(10)

            def refresh():
                sys.setprofile(observe_call)
                try:
                    outcome.append(reader.refresh())
                finally:
                    sys.setprofile(None)

            worker = threading.Thread(target=refresh)
            worker.start()
            try:
                assert entered.wait(10)
                assert reader.result.status == CollisionStatus.REVISION_PENDING
                assert reader.prepared_scene is None
                assert reader.device_transfer_count == 1
                if invalidate_during_prepare:
                    writer.invalidate()
            finally:
                release.set()
                worker.join(10)
            assert not worker.is_alive()
            assert outcome[0].status == (CollisionStatus.INVALID_SCENE if invalidate_during_prepare else CollisionStatus.OK)
            assert outcome[0].revision == notice
            if invalidate_during_prepare:
                assert reader.prepared_scene is None
            assert reader.device_transfer_count == 2


def test_abrupt_independent_writer_exit_revokes_an_existing_local_copy(shared_path):
    context = multiprocessing.get_context("spawn")
    parent, child = context.Pipe()
    process = context.Process(target=run_writer, args=(str(shared_path), child))
    process.start()
    try:
        assert parent.poll(20)
        parent.recv()
        parent.send("publish")
        assert parent.poll(20)
        parent.recv()
        safety = CollisionSafety(_config())
        with SharedSceneReader(shared_path, safety, _identities(safety.config), slot_timeout_ns=10_000_000_000) as reader:
            assert reader.refresh().status == CollisionStatus.OK
            parent.send("exit")
            process.join(10)
            assert process.exitcode == 17
            result = reader.refresh()
            assert result.status == CollisionStatus.INVALID_SCENE
            assert result.reason == "writer_exited"
            assert reader.prepared_scene is None
    finally:
        if process.is_alive():
            process.terminate()
            process.join(10)
        parent.close()
        child.close()


def test_revision_switches_during_real_copy_and_device_prepare_never_authorize_a_scene(shared_path):
    context = multiprocessing.get_context("spawn")
    parent, child = context.Pipe()
    process = context.Process(target=run_writer, args=(str(shared_path), child))
    process.start()
    try:
        assert parent.poll(20)
        parent.recv()
        parent.send("publish")
        assert parent.poll(20)
        first_notice = parent.recv()
        safety = CollisionSafety(_config())
        with SharedSceneReader(shared_path, safety, _identities(safety.config), slot_timeout_ns=10_000_000_000) as reader:
            assert reader.refresh().status == CollisionStatus.OK
            parent.send("burst")
            assert parent.poll(20)
            assert parent.recv() == "started"
            changed = []
            deadline = time.monotonic() + 10
            while not parent.poll() and time.monotonic() < deadline:
                result = reader.refresh()
                if result.reason.startswith("publication_changed"):
                    changed.append(result)
                    assert result.status == CollisionStatus.INVALID_SCENE
                    assert result.prepared_scene is None
                    assert reader.prepared_scene is None
                reader.reset_fault()
            assert parent.poll(10)
            assert parent.recv() == "finished"
            assert changed
            # 300 次切换后回到同一 slot；完整 revision 仍然核验变化。
            result = reader.refresh()
            if result.status == CollisionStatus.INVALID_SCENE:
                reader.reset_fault()
                result = reader.refresh()
            assert result.status == CollisionStatus.OK
            assert result.revision.revision == first_notice.revision + 300
            assert result.revision.active_slot == first_notice.active_slot
        parent.send("close")
        process.join(10)
        assert process.exitcode == 0
    finally:
        if process.is_alive():
            process.terminate()
            process.join(10)
        parent.close()
        child.close()
