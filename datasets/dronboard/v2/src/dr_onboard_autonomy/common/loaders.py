# pylint: disable=C0114
import os
from pathlib import Path
from typing import Any, TypeVar, Type, Dict, List, Union, Tuple, Optional
import logging


import tomli
from pydantic import BaseModel

M = TypeVar("M", bound=BaseModel)

Overrides = List[Tuple[Tuple[str, ...], Any]]

def load_model(
    model: Type[M],
    path: Union[Path, None] = None,
    overrides: Optional[Overrides] = None,
) -> M:
    """Read the settings from toml file and cache the result so the function can
    be called in multiple places without reparsing

    Example
    -------
    ```toml
    # universe.toml
    [hello]
    name = "greeting"
    age = 1
    world = { name = "earth", age = 2 }

    [goodbye]
    name = "adieu"
    age = 3
    world = { name = "mars", age = 2 }
    ```

    Given the above `universe.toml` file

    ```python
    class World(BaseModel):
        name: str
        age: int


    class Hello(BaseModel):
        name: str
        age: int
        world: World


    class Goodbye(BaseModel):
        name: str
        age: int
        world: World


    class Universe(BaseModel):
        hello: Hello
        goodbye: Goodbye


    os.environ["DR__GOODBYE__NAME"] = "farewell"
    os.environ["DR__HELLO__AGE"] = "6"
    load_model(Universe, Path("./universe.toml"), {})
    ```

    should result in

    ```python
    Universe(
        hello=Hello(
            name='greeting',
            age=6,
            world=World(name='earth', age=2)
        ),
        goodbye=Goodbye(
            name='farewell',
            age=3,
            world=World(name='mars', age=2)
        )
    )
    ```
    """
    # Replace tomli dep with stdlib once migrated to python 3.11

    from_file = tomli.loads(path.read_text()) if path is not None else {}

    with_env = load_env_overrides(from_file)

    with_overrides = override(with_env, overrides)

    return model(**with_overrides)


def load_env_overrides(
    init: Dict[str, Any], prefix: str = "DR__", sep: str = "__"
) -> Dict[str, Any]:
    for k, v in os.environ.items():
        if not k.startswith(prefix):
            continue
        logging.info("Setting config from env: %s=%s", k, v)
        keys: List[str] = [s.lower() for s in k.split(sep)][1:]
        nested_set(keys, v, init)

    return init


def override(
    init: Dict[str, Any],
    overrides: Optional[Overrides] = None,
):
    if overrides is not None:
        for k, v in overrides:
            nested_set(k, v, init)

    return init


def nested_set(keys: Union[tuple, list], value, init: dict):
    key = keys[0]
    if len(keys) > 1:
        value = nested_set(keys[1:], value, init.setdefault(key, {}))

    init[key] = value
    return init
