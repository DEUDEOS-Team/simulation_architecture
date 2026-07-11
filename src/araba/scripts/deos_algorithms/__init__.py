"""deos_algorithms — sanal paket köprüsü.

Algoritma modülleri birbirine `from deos_algorithms.xxx import ...` diye referans
verir; gerçek dosyalar ise bir üst dizinde (scripts/) durur. Eskiden bu dizinde
her modül için symlink vardı, ancak symlink'ler git/Windows üzerinden geçerken
içi "../xxx.py" yazan bozuk metin dosyalarına dönüştü (SyntaxError).

Bu __init__, symlink'e hiç ihtiyaç bırakmaz: bir "meta path finder" kaydeder ve
`deos_algorithms.X` import edildiğinde üst dizindeki gerçek `X` modülünü aynen
o isimle sunar (modül İKİ KEZ yüklenmez; sınıf kimlikleri/isinstance korunur).
`deos_algorithms.sensors.types` da aynı yolla gerçek `sensors.types`'a gider.
"""

import importlib
import importlib.abc
import importlib.machinery
import importlib.util
import os
import sys

# Üst dizin (scripts/) — gerçek modüllerin evi. Node'lar kendi başına çalışırken
# de bulunsun diye sys.path'e eklenir.
_SCRIPTS_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if _SCRIPTS_DIR not in sys.path:
    sys.path.insert(0, _SCRIPTS_DIR)

# Alt modül aramasının bu dizindeki (varsa) bozuk stub/symlink dosyalarına
# düşmemesi için paket yolu boşaltılır — her şey finder üzerinden çözülür.
__path__ = []

_PREFIX = __name__ + "."


class _AliasLoader(importlib.abc.Loader):
    """`deos_algorithms.X` için gerçek `X` modülünü döndürür (yeniden yüklemeden)."""

    def __init__(self, real_name):
        self._real_name = real_name

    def create_module(self, spec):
        return importlib.import_module(self._real_name)

    def exec_module(self, module):
        pass  # gerçek modül zaten çalıştırıldı


class _AliasFinder(importlib.abc.MetaPathFinder):
    def find_spec(self, fullname, path=None, target=None):
        if not fullname.startswith(_PREFIX):
            return None
        real_name = fullname[len(_PREFIX):]            # örn. "sensors.types"
        try:
            real_spec = importlib.util.find_spec(real_name)
        except (ImportError, ValueError):
            return None
        if real_spec is None:
            return None
        spec = importlib.machinery.ModuleSpec(fullname, _AliasLoader(real_name))
        if real_spec.submodule_search_locations is not None:   # gerçek bir paketse
            spec.submodule_search_locations = list(real_spec.submodule_search_locations)
        return spec


# Yerleşik path-finder'lardan ÖNCE çalışsın diye başa eklenir (bir kez).
if not any(isinstance(f, _AliasFinder) for f in sys.meta_path):
    sys.meta_path.insert(0, _AliasFinder())
