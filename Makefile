PY ?= python3
# Full verification, in dependency order. Each step exits non-zero on a failed requirement.
verify: mech propulsion fea flight

mech:        ; $(PY) -m mech.checks
propulsion:  ; $(PY) -m prop.propulsion
fea:         ; $(PY) -m mech.fea
flight:      ; $(PY) -m flight.fly --n 256
export:      ; $(PY) -m mech.export
retrain:     ; $(PY) -m flight.retrain
test:        ; $(PY) -m pytest -q tests

.PHONY: verify mech propulsion fea flight export retrain test
