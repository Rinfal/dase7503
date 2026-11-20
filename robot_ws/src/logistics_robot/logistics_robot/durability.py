"""Commit local files atomically, flushing data before publishing the filename."""
import os
import tempfile
from contextlib import contextmanager
from pathlib import Path


def sync_directory(directory):
    fd = os.open(directory, os.O_RDONLY | os.O_DIRECTORY)
    try:
        os.fsync(fd)
    finally:
        os.close(fd)


@contextmanager
def atomic_path(destination):
    destination = Path(destination)
    fd, name = tempfile.mkstemp(prefix='.'+destination.stem+'-', suffix=destination.suffix,
                                dir=destination.parent)
    os.close(fd)
    temporary = Path(name)
    try:
        yield temporary
        with temporary.open('rb') as stream:
            os.fsync(stream.fileno())
        os.replace(temporary, destination)
        sync_directory(destination.parent)
    finally:
        temporary.unlink(missing_ok=True)


def atomic_write(destination, data):
    with atomic_path(destination) as temporary:
        temporary.write_bytes(data)
