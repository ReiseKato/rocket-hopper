# Installation und lokale Entwicklung

## Voraussetzungen

Benötigt werden:

- Git,
- Python 3.11 oder neuer,
- `pip` und `venv` (normalerweise Bestandteil einer Python-Installation).

Die erste Version des physikalischen Kerns benötigt keine NumPy-Abhängigkeit.

## Empfohlene virtuelle Umgebung

Vom Root-Verzeichnis des `rocket-hopper`-Repositories:

=== "Linux / macOS"

    ```bash
    python3 -m venv .venv
    source .venv/bin/activate
    python -m pip install --upgrade pip
    python -m pip install -e "./fast_sim[dev,docs]"
    ```

=== "Windows PowerShell"

    ```powershell
    py -m venv .venv
    .\.venv\Scripts\Activate.ps1
    python -m pip install --upgrade pip
    python -m pip install -e ".\fast_sim[dev,docs]"
    ```

Die Extras installieren:

- `dev`: `pytest`,
- `docs`: `mkdocs-material` inklusive kompatibler MkDocs- und Markdown-Abhängigkeiten.

## Tests ausführen

```bash
python -m pytest fast_sim/tests
```

## Dokumentation lokal anzeigen

Vom Repository-Root:

```bash
mkdocs serve
```

MkDocs startet einen lokalen Entwicklungsserver und aktualisiert die Dokumentation bei Änderungen automatisch.

## Paket im Editable Mode

Die Option `-e` installiert den Fast Simulator im sogenannten Editable Mode. Änderungen unter `fast_sim/src/` werden dadurch direkt von der virtuellen Umgebung verwendet, ohne nach jeder Änderung neu installieren zu müssen.
