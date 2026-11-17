# rlval — 실게임 대비 시뮬레이터 검증 harness

RLBot v5로 실제 Rocket League에서 시나리오를 녹화하고, 같은 초기 상태와 입력을
시뮬레이터 backend에서 재생해 궤적과 bump/demo 결과를 비교한다.
backend는 C++ 포크(`RocketSim` Python 모듈)와 RocketSim v3(Rust) 두 가지다.

## 구성

| 경로 | 역할 |
|---|---|
| `rlval/scenarios.py` | 시나리오 정의 (주행, 점프·flip, 공, bump·demo) |
| `rlval/record.py` | RLBot v5 script. 게임에서 시나리오를 실행하고 모든 패킷을 녹화 |
| `rlval/recording.py` | 녹화 파일 형식 (`.npz`, 시나리오 1회당 1개) |
| `rlval/job.py` | 녹화 → backend 공통 재생 job(JSON) 변환 |
| `rlval/backend_cpp.py` | C++ 포크 backend. 현재 venv에 설치된 `RocketSim` 모듈을 쓴다 |
| `rlval/backend_v3.py`, `replay_v3/` | v3 backend. Rust CLI가 job을 stdin으로 받아 결과를 stdout으로 낸다 |
| `rlval/compare.py` | 녹화를 backend에서 재생하고 오차 표를 출력 |
| `rlval/synth.py` | 게임 대신 시뮬레이터로 녹화를 만들어 파이프라인과 backend 간 차이를 확인 |
| `rlval/outcomes.py` | 차 간 충돌 시나리오의 결과(demo/bump/없음)를 backend별로 나란히 출력 |
| `rlbot/` | 매치 설정과 매치 시작 스크립트 |

## 준비

```sh
uv venv --python 3.12 && uv pip install --prerelease allow -e . .. pytest
(cd replay_v3 && cargo build --release)
```

- 이 디렉터리는 RocketSim 포크 안에 있다. collision mesh는 포크 옆의 `collision_meshes`(`../../collision_meshes`)를 쓴다 (`RLVAL_MESHES`로 변경 가능).
- v3는 포크 옆의 `rocketsim-v3` checkout(ZealanL/RocketSim `v3-rust`, tag `v0.3.6`)을 path dependency로 쓰고 `native-math` feature를 켠다.
- 포크의 다른 브랜치를 비교하려면 venv를 따로 만들어 그 브랜치를 설치한다.
  예: `.venv-demo-final`에는 `angle-supersonic-forward-backward-forwardvel`이 들어 있다.

## 실게임 녹화

게임과 `RLBotServer`는 게임을 소유한 계정에서 띄우고, 녹화 script는 어느 계정에서든 접속한다.

1. 게임 소유 계정에서 `RLBotServer`를 실행한다.
2. 매치를 시작한다. 서버가 설정 파일을 직접 읽으므로 그 계정이 읽을 수 있는 경로여야 한다.
   ```sh
   python rlbot/start_match.py
   ```
3. 녹화한다. 인자는 시나리오 이름이나 태그의 부분 문자열이고, 없으면 전체다.
   ```sh
   python -m rlval.record --list
   python -m rlval.record --out recordings/ delay
   python -m rlval.record --out recordings/ --repeats 3 car_car
   ```

매치는 봇 프로세스가 없는 차 2대로 구성된다. state setting이 켜져 있으면 서버가
script의 입력을 모든 차에 대해 받아 주므로, 녹화 script 하나가 두 차를 모두 조종한다.

## 비교

```sh
python -m rlval.compare recordings/ --backends cpp v3 --fit-delay --csv result.csv
```

- 실행 구간 시작 시점의 녹화 상태에서 open loop로 재생하고 0.25/0.5/1/2초와 끝 시점의 위치 오차를 낸다.
- `--delay N`은 패킷 i→i+1 구간에 패킷 i+N의 `last_input`을 쓴다는 뜻이다.
  `--fit-delay`는 -1..4를 모두 시도해 녹화별로 가장 잘 맞는 값을 고른다.
- 녹화에 demo가 있었는지와 시뮬레이터가 demo를 냈는지를 함께 보여 준다.

## 게임 없이 확인

```sh
python -m pytest -q tests                        # 녹화 script를 가짜 패킷으로 구동
python -m rlval.synth --backend cpp --out recordings_synth/cpp
python -m rlval.compare recordings_synth/cpp --backends cpp v3
python -m rlval.outcomes --backend v3 --label v3 --out outcomes/
python -m rlval.outcomes --merge outcomes/*.json
```

## 알려진 한계

- RLBot의 state setting은 물리 상태와 boost만 설정한다. 그래서 시나리오는 지면에 놓인 중립 상태나
  속도를 직접 준 상태에서 시작하고, 점프·flip 같은 내부 상태는 입력으로 만들어 낸다.
- 재생 시작 시 handbrake 아날로그 값, bump cooldown, supersonic 유지 타이머 같은 숨은 상태는 기본값이다.
- 녹화 script는 실게임에서 아직 돌려 보지 않았다. 가짜 패킷 테스트만 통과한 상태다.
- 입력 지연은 문서화된 값이 없어 `input_delay_*` 시나리오로 측정해야 한다.
