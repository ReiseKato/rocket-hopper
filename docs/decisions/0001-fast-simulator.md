# ADR 0001: Fast Simulator im gemeinsamen Repository

## Status

Akzeptiert.

## Entscheidung

Der Fast Simulator wird als eigener Bereich `fast_sim/` im gemeinsamen `rocket-hopper`-Repository geführt.

## Begründung

Der Simulator ist ein zentraler Bestandteil desselben technischen Systems wie spätere Hardware-, Systemidentifikations- und MuJoCo-Komponenten. Ein gemeinsames Repository erleichtert abgestimmte Änderungen und eine zentrale Dokumentation.

Der Fast Simulator bleibt trotzdem als klar abgegrenztes Python-Projekt strukturiert.
