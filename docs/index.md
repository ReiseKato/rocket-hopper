# Rocket Hopper

Diese Dokumentation beschreibt das Rocket-Hopper-Projekt als technische Entwicklungsplattform. Das langfristige Ziel ist ein kleiner wiederverwendbarer Hopper mit zentralem Hauptantrieb und Schubvektorsteuerung, dessen Simulation schrittweise durch reale Messdaten verbessert wird.

## Aktueller Schwerpunkt

Der erste konkrete Baustein ist der **Fast Simulator**. Er soll bewusst klein, schnell und mathematisch transparent bleiben. Die erste Version beschreibt einen zweidimensionalen Starrkörper mit einem gegimbal-ten Hauptschub.

Die Dokumentation ist nach Konzepten gegliedert und nicht als chronologisches Projekttagebuch. Dadurch sollen Definitionen, Gleichungen, Architekturentscheidungen und Validierungsfälle später schnell auffindbar bleiben.

## Leitprinzipien

- Physik vor Komplexität.
- Unbekannte Hardwarewerte bleiben Parameter.
- Modell, Aktuatoren, Controller und numerische Integration werden getrennt.
- Jede Erweiterung wird gegen bekannte Spezialfälle validiert.
- Der Fast Simulator bleibt unabhängig von MuJoCo und anderen großen Frameworks.
