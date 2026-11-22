PY ?= python3
# Full verification, in dependency order. Each step exits non-zero on a failed requirement.
verify: mech propulsion fea flight

mech:        ; $(PY) -m mech.checks
propulsion:  ; $(PY) -m prop.propulsion
fea:         ; $(PY) -m mech.fea
flight:      ; $(PY) -m flight.fly --n 256
export:      ; $(PY) -m mech.export
retrain:     ; $(PY) -m flight.retrain
test:        ; $(PY) -m pytest -q ecad/check_design.py ecad/check_layout.py && $(MAKE) lite-test lite-board

# NF-3D Lite (low-budget build). lite-verify runs everything that needs no hardware; the slow
# closed-loop simulations (autonomy, vision) take tens of minutes on 4 cores: run them one at a time.
lite-verify: lite-test lite-checks lite-detail lite-board lite-fly lite-stress lite-autonomy lite-vision
lite-test:     ; bash lite/firmware/tests/run.sh && $(PY) -m pytest -q lite/ground/tests
lite-firmware: ; bash lite/firmware/build.sh
lite-checks:   ; $(PY) -m lite.checks
lite-detail:   ; $(PY) -m lite.detail
lite-board:    ; cd lite/ecad && $(PY) -m pytest -q check_board.py
lite-fly:      ; $(PY) -m lite.fly --n 256
lite-stress:   ; $(PY) -m lite.stress
lite-autonomy: ; $(PY) -u -m lite.autonomy --n 256
lite-vision:   ; $(PY) -u -m lite.vision_sim --n 8
lite-learned:  ; $(PY) -u -m lite.learned --n 128
lite-export:   ; $(PY) -m lite.export

.PHONY: verify mech propulsion fea flight export retrain test lite-verify lite-test lite-firmware lite-checks \
        lite-detail lite-board lite-fly lite-stress lite-autonomy lite-vision lite-learned lite-export
