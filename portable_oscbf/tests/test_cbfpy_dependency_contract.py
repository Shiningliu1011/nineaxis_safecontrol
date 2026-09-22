import ast
import importlib.metadata
from pathlib import Path

from cbfpy import CBF, CBFConfig
from packaging.requirements import Requirement

from work.nineaxis_manipulator_jax import NineaxisManipulatorJAX
from work.oscbf_velocity_config import NineaxisOSCBFVelocityConfig


REPO_ROOT = Path(__file__).resolve().parents[2]
PINNED_DEPENDENCIES = {
    "cbfpy": "0.0.1",
    "jax": "0.6.2",
    "jaxlib": "0.6.2",
    "qpax": "0.1.4",
}
CBFPY_MEMBERS = (
    "m",
    "num_cbf",
    "relax_cbf",
    "cbf_relaxation_penalty",
    "solver_tol",
    "P_qp",
    "q_qp",
    "G_qp",
    "h_qp",
    "qp_solver",
)


def _requirements_from_manifest() -> dict[str, Requirement]:
    manifest = REPO_ROOT / "portable_oscbf" / "requirements.txt"
    requirements = [
        Requirement(line)
        for line in manifest.read_text(encoding="utf-8").splitlines()
        if line
    ]
    return {requirement.name: requirement for requirement in requirements}


def _requirements_from_setup() -> dict[str, Requirement]:
    setup_tree = ast.parse(
        (REPO_ROOT / "setup.py").read_text(encoding="utf-8"),
        filename="setup.py",
    )
    assignment = next(
        node for node in setup_tree.body
        if isinstance(node, ast.Assign)
        and any(
            isinstance(target, ast.Name)
            and target.id == "PORTABLE_OSCBF_RUNTIME_REQUIREMENTS"
            for target in node.targets
        )
    )
    requirements = [Requirement(value) for value in ast.literal_eval(assignment.value)]
    return {requirement.name: requirement for requirement in requirements}


def test_dependency_manifest_matches_install_configuration():
    manifest = _requirements_from_manifest()
    install_configuration = _requirements_from_setup()

    assert manifest == install_configuration
    assert {
        name: str(requirement.specifier)
        for name, requirement in manifest.items()
    } == {
        name: f"=={version}"
        for name, version in PINNED_DEPENDENCIES.items()
    }


def test_pinned_runtime_dependency_versions_are_installed():
    installed = {
        name: importlib.metadata.version(name)
        for name in PINNED_DEPENDENCIES
    }

    assert installed == PINNED_DEPENDENCIES


def test_cbfpy_interface_contract():
    assert issubclass(NineaxisOSCBFVelocityConfig, CBFConfig)

    config = NineaxisOSCBFVelocityConfig(NineaxisManipulatorJAX())
    cbf = CBF.from_config(config)

    assert all(hasattr(cbf, member) for member in CBFPY_MEMBERS)
