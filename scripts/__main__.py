
import asyncio
import importlib
import inspect
import os
import stat
import subprocess
import sys
from pathlib import Path
from typing import Any, Dict, List, Optional

import click
import inquirer

from ._utils import title

AnyDict = Dict[str, Any]

BACK_CHOICE = "<Back>"
EXIT_CHOICE = "<Exit>"

SCRIPTS_DIR = 'scripts'

EXT_MAP = {
    "py": "python3",
    "mjs": "zx",
    "sh": "bash",
}


def make_scripts_dict(base_dir: Path) -> AnyDict:
    scripts_dict = {}
    for dirname, _, files in os.walk(base_dir):
        if dirname.endswith('/utils'):
            continue
        scripts = list(
            filter(
                lambda filename: (
                    not filename.startswith("__")
                    and any(map(lambda ext: filename.endswith(f".{ext}"), EXT_MAP.keys()))
                ),
                files,
            )
        )
        if scripts:
            scripts_dict[dirname] = scripts
    return scripts_dict


def choose_dir(scripts_dict: AnyDict, base_dir: Path) -> str:
    choices = list(map(lambda directory: directory[len(str(base_dir)) + 1 :], scripts_dict.keys()))
    choices.append(EXIT_CHOICE)

    prompt = "Type"
    questions = [
        inquirer.List(
            prompt,
            message="Choose type",
            choices=choices,
        ),
    ]
    return inquirer.prompt(questions)[prompt]


def choose_script(scripts: List[str]) -> str:
    prompt = "Script"
    scripts.append(BACK_CHOICE)
    questions = [
        inquirer.List(
            prompt,
            message="Choose script",
            choices=scripts,
        ),
    ]
    return inquirer.prompt(questions)[prompt]


def run_py_func(function) -> None:
    if inspect.iscoroutinefunction(function):
        loop = asyncio.get_event_loop()
        loop.run_until_complete(function())
        loop.close()
    else:
        function()


def run_script(path_to_script: Path) -> None:
    with open(str(path_to_script), "r") as script_file:
        executable = script_file.readline().startswith("#!")

    script = str(path_to_script.relative_to(Path(sys.argv[0]).parent))
    exec_script_path = SCRIPTS_DIR + '/' + script

    base_package_path, script_module_path = script.split("/")
    script_module_path = script_module_path.replace(".py", "")

    print("Running script: ", script)
    print(title(script_module_path))

    if executable:
        try:
            subprocess.call(exec_script_path)
        except PermissionError:
            os.chmod(exec_script_path, os.stat(exec_script_path).st_mode | stat.S_IEXEC)
            subprocess.call(
                exec_script_path,
                env={"PYTHONPATH": f"{os.getenv('PYTHONPATH')}"},)
    else:
        if '.' not in script:
            raise ValueError(f"{script} does not have a file extension")
        extension = script.split('.')[1]
        if extension not in EXT_MAP:
            raise ValueError(f"Unsupported extenstion {extension}")
        executor = EXT_MAP[extension]
        if executor == "python3":
            base_package = importlib.import_module(base_package_path)
            base_package.validate()  # type:ignore

            script_module = importlib.import_module(f"{base_package_path}.{script_module_path}")
            run_py_func(script_module.main)  # type:ignore
        elif executor == 'bash':
            subprocess.call(["bash", exec_script_path])
        else:
            raise ValueError(f"Unsupported executor {executor}")


def get_script(base_dir: Path) -> Path:
    script_ = None

    while script_ is None or script_ == BACK_CHOICE:
        scripts_dict = make_scripts_dict(base_dir)

        dir_ = choose_dir(scripts_dict, base_dir)
        if dir_ == EXIT_CHOICE:
            sys.exit(0)
        directory = base_dir / dir_

        script_ = choose_script(scripts_dict[str(directory)])

    script = directory / script_
    return script


@click.command()
@click.argument(
    "script",
    nargs=1,
    required=False,
)
def run(script: Optional[str] = None) -> None:
    """Run service scripts easily"""
    base_dir = Path(sys.argv[0]).parent
    sys.path.append(str(base_dir))
    if script:
        to_run = base_dir / script
    else:
        to_run = get_script(base_dir)
    run_script(to_run)


if __name__ == "__main__":
    run()
