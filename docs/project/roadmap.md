# Roadmap

Die Entwicklung erfolgt bewusst in kleinen, validierbaren Stufen.

## Phase 1 — 2D-Rigid-Body-Modell

Aktueller Fokus:

1. Koordinatensystem und Vorzeichen eindeutig definieren.
2. Zustand, physikalische Inputs und Parameter modellieren.
3. Newton-Euler-Dynamik implementieren.
4. Physiktests gegen analytische Spezialfälle schreiben.
5. Numerischen Integrator ergänzen.
6. Open-Loop-Simulationen durchführen.
7. Einfache Lage- und Höhenregelung hinzufügen.
8. Gekoppelte Positionsregelung und einfache Landung untersuchen.

## Später

- realistischere Aktuatormodelle,
- Sensoren und Zustandsschätzung,
- Parameterstudien und Monte-Carlo,
- 3D-6-DoF-Modell,
- MuJoCo-Backend,
- Systemidentifikation aus Messdaten,
- Vergleich realer Logs mit Simulation.

Neue Komplexität wird erst ergänzt, wenn die vorherige Modellstufe verstanden und validiert ist.
