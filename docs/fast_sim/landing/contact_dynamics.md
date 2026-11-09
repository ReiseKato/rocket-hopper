# Minimale Kontaktdynamik

## Zweck

Nach dem ersten Fußkontakt darf der Hopper nicht mehr mit der freien Starrkörperdynamik weitergerechnet werden.

Der erste Kontakt erzeugt einen Bodenimpuls und schränkt die anschließende Bewegung ein.

Für Landing V0 wird deshalb ein bewusst einfaches Kontaktmodell eingeführt:

> Der zuerst aufsetzende Fuß haftet nach einem vollständig unelastischen Stoß als fester Punkt am Boden.

Das Modell dient ausschließlich dazu, eine physikalisch nachvollziehbare Bewegung vom ersten bis zum zweiten Fußkontakt zu erhalten.

Es ist kein vollständiges Fahrwerks- oder Kontaktmodell.

## Kontaktfolge

Das Landemanöver kann damit in folgende physikalische Modi zerlegt werden:

```text
FREE_FLIGHT
    │
    │ erster Fußkontakt
    ▼
LEFT_CONTACT oder RIGHT_CONTACT
    │
    │ Rotation um festen Fuß
    ▼
BOTH_CONTACT
```

`BOTH_CONTACT` bezeichnet den geometrisch abgeschlossenen Landevorgang.

Ob die Landung erfolgreich ist, wird später durch separate Erfolgskriterien bewertet.

## Annahmen der ersten Version

Der erste Fußkontakt wird als:

- vollständig unelastisch,
- haftend,
- punktförmig,
- ohne Schlupf,
- ohne Abheben

modelliert.

Der Kontaktpunkt bleibt nach dem Stoß fest:

\[
x_C=\text{const}
\]

und:

\[
z_C=z_{\mathrm{ground}}.
\]

Das Modell besitzt bewusst keine:

- Federsteifigkeit,
- Dämpfung,
- Reibungszahl,
- Kontaktzeit,
- elastische Rückfederung.

Damit müssen keine unbekannten Hardwareparameter erfunden werden.

## Erster Stoß

Unmittelbar vor dem ersten Kontakt besitzt der freie Starrkörper:

\[
v_x^-,
\qquad
v_z^-,
\qquad
\omega^-.
\]

Sei:

\[
r=
\begin{bmatrix}
r_x\\
r_z
\end{bmatrix}
\]

der Vektor vom Schwerpunkt zum kontaktierenden Fuß im World Frame.

Während des idealisierten instantanen Stoßes wird der Drehimpuls um den Kontaktpunkt erhalten.

Der Bodenimpuls greift direkt im Kontaktpunkt an und besitzt deshalb dort keinen Hebelarm.

Der Drehimpuls vor dem Stoß lautet:

\[
H_C^-
=
I_y\omega^-
+
m
\left(
r_xv_z^-
-r_zv_x^-
\right).
\]

## Trägheitsmoment um den Kontaktpunkt

Nach dem Steiner-Satz gilt:

\[
I_C
=
I_y
+
m(r_x^2+r_z^2).
\]

Da die Fahrwerksgeometrie starr ist:

\[
r_x^2+r_z^2
=
\left(\frac b2\right)^2+h^2.
\]

Damit ist \(I_C\) für beide symmetrischen Füße gleich.

## Winkelgeschwindigkeit nach dem Stoß

Nach dem Stoß bewegt sich der Hopper nur noch als Rotation um den festen Kontaktpunkt.

Daher gilt:

\[
H_C^+
=
I_C\omega^+.
\]

Aus:

\[
H_C^-=H_C^+
\]

folgt:

\[
\boxed{
\omega^+
=
\frac{
I_y\omega^-
+
m(r_xv_z^--r_zv_x^-)
}{
I_C
}
}.
\]

## Schwerpunktgeschwindigkeit nach dem Stoß

Da der Kontaktpunkt feststeht, wird die Schwerpunktgeschwindigkeit vollständig durch die Rotation um diesen Punkt bestimmt.

Mit dem Vektor vom Kontaktpunkt zum Schwerpunkt:

\[
q=-r
\]

gilt:

\[
v_x^+
=
\omega^+q_z
\]

und:

\[
v_z^+
=
-\omega^+q_x.
\]

Äquivalent:

\[
v_x^+
=
-\omega^+r_z
\]

und:

\[
v_z^+
=
\omega^+r_x.
\]

Damit besitzt der kontaktierende Fuß unmittelbar nach dem Stoß die Geschwindigkeit:

\[
v_C^+=0.
\]

## Projektion auf die Bodenebene

Durch die diskrete Simulation kann der erkannte Fuß beim ersten gespeicherten Kontaktzustand bereits geringfügig unterhalb des Bodens liegen.

Beim Übergang in den Single-Contact-Modus wird deshalb der Kontaktpunkt auf:

\[
z_C=z_{\mathrm{ground}}
\]

gesetzt.

Der horizontale Kontaktort wird aus der zum Kontaktzeitpunkt berechneten Fußposition übernommen.

Die Fahrzeuglage \(\theta\) bleibt beim instantanen Stoß unverändert.

## Energie des ersten Stoßes

Vor dem Kontakt besitzt das Fahrzeug die kinetische Energie:

\[
E_1^-
=
\frac12m(v_x^2+v_z^2)
+
\frac12I_y\omega^2.
\]

Nach dem Stoß gilt:

\[
E_1^+
=
\frac12I_C(\omega^+)^2.
\]

Die im idealisierten vollständig unelastischen Stoß dissipierte Energie lautet:

\[
\boxed{
E_{\mathrm{diss},1}
=
E_1^-
-
E_1^+
}.
\]

Diese Größe ist eine Aussage innerhalb des gewählten Kontaktmodells.

Sie ist nicht automatisch die reale Energieaufnahme eines einzelnen Fahrwerkbeins.

## Single-Contact-Zustand

Nach dem ersten Stoß besitzt das System nur noch einen Freiheitsgrad.

Der reduzierte Zustand lautet:

\[
x_C
=
\begin{bmatrix}
\theta\\
\omega
\end{bmatrix}.
\]

Die translatorischen Schwerpunktgrößen sind durch den festen Kontaktpunkt und die Starrkörpergeometrie vollständig bestimmt.

`SingleContactState2D` enthält deshalb nur:

- `theta`,
- `omega`.

Die feste Kontaktbedingung wird separat durch `SingleContactConstraint2D` beschrieben.

Sie enthält:

- den kontaktierenden Fuß,
- die feste horizontale Kontaktposition,
- die Bodenhöhe.

## Rekonstruktion des Starrkörperzustands

Für den Kontaktpunkt und die aktuelle Fahrzeuglage wird zunächst der Vektor:

\[
q=
\begin{bmatrix}
q_x\\
q_z
\end{bmatrix}
\]

vom Kontaktpunkt zum Schwerpunkt bestimmt.

Dann gilt:

\[
p_x=x_C+q_x
\]

\[
p_z=z_{\mathrm{ground}}+q_z
\]

\[
v_x=\omega q_z
\]

\[
v_z=-\omega q_x.
\]

Damit kann zu jedem Zeitpunkt wieder ein vollständiger `HopperState2D` erzeugt werden.

Das ist wichtig, weil bestehende Controller und die Touchdown Detection weiterhin mit `HopperState2D` arbeiten.

## Rotationsdynamik um den festen Fuß

Während des Single-Contact-Modus gilt:

\[
I_C\dot\omega
=
\tau_C.
\]

Die Bodenreaktion muss in dieser Momentengleichung nicht explizit auftreten, weil sie direkt im festen Kontaktpunkt angreift.

Berücksichtigt werden:

- Gravitation,
- Schubkraft,
- das Gimbal-bedingte Schubmoment um den Schwerpunkt.

## Gravitation

Für den Vektor vom Kontaktpunkt zum Schwerpunkt:

\[
q=
\begin{bmatrix}
q_x\\
q_z
\end{bmatrix}
\]

lautet das Gravitationsmoment um den Kontaktpunkt:

\[
\tau_g
=
mgq_x.
\]

Das Vorzeichen folgt aus der bestehenden \(+y\)-Drehkonvention.

## Schub

Die Schubkraft im World Frame lautet weiterhin:

\[
F_T^W
=
\begin{bmatrix}
T\sin(\theta+\delta)\\
T\cos(\theta+\delta)
\end{bmatrix}.
\]

Das Moment dieser Kraft um den Kontaktpunkt, wenn sie zunächst am Schwerpunkt betrachtet wird, lautet:

\[
\tau_{T,C}
=
q_zF_{T,x}
-
q_xF_{T,z}.
\]

Zusätzlich bleibt das bekannte Moment durch den Engine-Offset bestehen:

\[
\tau_{\mathrm{offset}}
=
-lT\sin\delta.
\]

Damit:

\[
\boxed{
I_C\dot\omega
=
mgq_x
+
q_zT\sin(\theta+\delta)
-
q_xT\cos(\theta+\delta)
-
lT\sin\delta
}.
\]

## Aktuation während des Kontakts

Das Kontaktmodell schaltet Schub und Gimbal nicht automatisch ab.

Es erhält weiterhin eine physikalische `HopperActuation2D`.

Damit bleibt die Verantwortungsgrenze bestehen:

```text
Controller / Aktuatormodelle
        ↓
HopperActuation2D
        ↓
SingleContactDynamics2D
```

Ob die Landing Guidance oder eine spätere Landing-Orchestrierung nach dem ersten Fußkontakt andere Commands erzeugt, ist keine Entscheidung der Kontaktdynamik.

## Zweiter Fußkontakt

Aus dem reduzierten Single-Contact-Zustand wird kontinuierlich ein vollständiger `HopperState2D` rekonstruiert.

Der bestehende `TouchdownDetector2D` kann darauf erneut angewendet werden.

Sobald er:

```python
TouchdownContact2D.BOTH
```

liefert, hat auch der zweite Fuß den Boden erreicht.

Damit ist die Landung geometrisch abgeschlossen.

## Noch keine numerische Orchestrierung

`SingleContactDynamics2D` beschreibt die momentane reduzierte Dynamik:

\[
(\theta,\omega,u)
\rightarrow
(\dot\theta,\dot\omega).
\]

Die Klasse enthält bewusst keinen eigenen Zeitschleifen- oder Moduswechselcode.

Die spätere Landing-Orchestrierung entscheidet:

```text
Free-flight dynamics
        ↓ first contact
Impact transition
        ↓
Single-contact dynamics
        ↓ second contact
Landing completed
```

Dadurch bleibt die Kontaktdynamik unabhängig von der konkreten Simulationssteuerung.

## Grenzen des Modells

Das aktuelle Modell setzt voraus, dass der erste Kontaktpunkt:

- nicht rutscht,
- nicht abhebt,
- keine elastische Verformung besitzt.

Damit können noch keine realen:

- Kontaktkräfte,
- g-Belastungen,
- Federwege,
- strukturellen Lastspitzen,
- Reibungsgrenzen

bestimmt werden.

Für Landing V0 reicht das Modell jedoch aus, um den ersten Stoß physikalisch auf den Starrkörperzustand wirken zu lassen und anschließend einen sinnvollen zweiten Fußkontakt zu erreichen.
