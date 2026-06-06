# Ke hoach nang cap lop truyen thong CBTC

Tai lieu nay phan tich yeu cau nang cap lop truyen thong cho mo phong CBTC hien tai. Muc tieu la giu nguyen cac chuc nang dang chay, nhung tach ro hon cac tang truyen tin: DCS chi van chuyen goi tin, RaSTA-like dung cho ban tin an toan, OPC UA-like dung cho giam sat/van hanh khong vital.

## 1. Hien trang trong du an

Hien tai du an da co cac thanh phan nen:

- `ZoneController` tinh Movement Authority / EOA va tao `SafeMovementPacket`.
- `OnboardControlCenter` nhan packet theo thoi gian den, luu packet moi nhat va cap nhat `eoa`, `psr_kmh`, speed limit cho train.
- `DCSWatchdog` kiem tra timeout va packet integrity co ban.
- Runtime tao delay ngau nhien cho safe packet bang `DCS_DELAY_MIN_S` va `DCS_DELAY_MAX_S`.
- Mat DCS co the lam `safe_packet_valid=False`, dan den ATP fail-safe logic.
- Dataflow Monitor hien thi luong ATS/ZC/DCS/CC/ATP/ATO o muc so do, chua phai bang packet-level.

Gioi han hien tai:

- Chua co Red/Blue network path.
- Chua co Radio Access Point theo vi tri tau.
- Chua co handover giua RAP.
- Chua co packet frame voi header, CRC, HMAC, TTL, session, sequence.
- Chua tach ro OPC UA-like va RaSTA-like.
- Chua co rejection reason chi tiet cho tung packet trong Dataflow.

## 2. Phan co the thuc hien duoc trong code hien tai

Nhung muc duoi day co the lam theo tung buoc ma khong can viet lai toan bo simulator.

### 2.1. Tao message schema ro rang

Co the them cac dataclass moi:

- `MovementAuthorityMessage`
- `PositionReportMessage`
- `TrainStatusMessage`
- `AtsOperationCommandMessage`
- `DcsHealthMessage`

Tac dung:

- Lam ro payload cua tung loai ban tin.
- Giam viec truyen dict tuy tien.
- Lam nen cho frame RaSTA/OPC UA sau nay.

### 2.2. Tao VitalSafePacket / SafetyFrame

Co the them lop moi, vi du:

- `VitalPacketHeader`
- `VitalPacketSafety`
- `VitalSafePacket`
- `VitalPacketValidator`

Packet co the gom:

- `protocol_version`
- `message_type`
- `source_id`
- `destination_id`
- `session_id`
- `sequence_number`
- `timestamp_ms`
- `ttl_ms`
- `payload_length`
- `payload`
- `crc32`
- `hmac_sha256`
- `packet_uuid`
- `key_id`

Co the kiem tra:

- Packet het TTL.
- Sai source/destination.
- Sai session.
- Duplicate packet.
- Sequence out-of-order.
- CRC sai.
- HMAC sai.
- Payload length sai.

Neu reject:

- Khong cap nhat MA/EOA.
- Khong cap nhat position report.
- Ghi log ly do.
- Gui ly do sang Dataflow Monitor.

### 2.3. Tach RaSTA-like va OPC UA-like trong code

Co the tao 2 module rieng:

- `SUBSYSTEMS/protocols/rasta_vital.py`
- `SUBSYSTEMS/protocols/opcua_supervision.py`

RaSTA-like dung cho:

- `ZC -> Train`: `MA_UPDATE`, `EOA`, `SPEED_PROFILE`.
- `Train -> ZC`: `POSITION_REPORT`, `TRAIN_INTEGRITY`, `SAFE_POSITION`.
- Emergency safety message.

OPC UA-like dung cho:

- `ATS -> Train/ATO`: hold, dwell extension, station skip, regulation command.
- `Train -> ATS`: train status, door status, mode, delay, fault state.
- `ATS <-> ZC/DCS`: diagnostics, network health.

Luat quan trong:

- OPC UA-like khong duoc cap MA/EOA.
- MA/EOA chi di qua RaSTA-like vital path.

### 2.4. Nang DCS thanh transport layer co Red/Blue

Co the thay DCS hien tai bang lop transport rieng:

- `DcsNetworkPath`: RED hoac BLUE.
- `DcsTransport`: chon path, failover, tinh latency/loss.
- `RadioAccessPoint`: vung phu song tren track.
- `DcsPacketEvent`: record cho Dataflow.

Trang thai network:

- `OK`: it mat goi, latency trong nguong.
- `DEGRADED`: latency cao, jitter cao, packet loss tang.
- `LOST`: khong gui duoc packet.

Failover:

- Neu RED loi thi chuyen sang BLUE.
- Neu BLUE loi thi chuyen sang RED.
- Neu ca hai loi thi vital channel lost.

### 2.5. RAP coverage va handover

Co the them cau hinh RAP:

```yaml
radio_access_points:
  - id: RAP_01
    start_m: 0
    end_m: 900
  - id: RAP_02
    start_m: 800
    end_m: 1700
```

Logic co the lam:

- Xac dinh RAP hien tai theo vi tri tau.
- Neu tau chuyen tu RAP nay sang RAP khac thi tao handover event.
- Gan path label vao packet: `RED+RAP_04`, `BLUE+RAP_04`.
- Gan mat song neu tau ngoai coverage.
- Tang packet loss/jitter khi tau gan mep coverage.

### 2.6. Dataflow packet-level

Co the nang Dataflow Monitor thanh bang co cot:

```text
time | from | to | protocol | path | msg_type | seq | latency | ttl | result | action | reason
```

Vi du:

```text
12.450 | ZC_01 | TRAIN_03 | RASTA_VITAL | RED+RAP_04 | MA_UPDATE | 1028 | 82ms | OK | ACCEPTED | MA updated
13.100 | ZC_01 | TRAIN_03 | RASTA_VITAL | RED+RAP_04 | MA_UPDATE | 1028 | 90ms | OK | REPLAY | ignored
13.500 | ATS | TRAIN_03 | OPCUA_SUPERVISION | RED | DWELL_EXTEND | 88 | 30ms | OK | ACCEPTED | dwell +15s
```

### 2.7. Fault injection co the them

Co the them cac fault sau:

- RED network failure.
- BLUE network failure.
- Both network failure.
- Radio coverage loss.
- RAP handover failure.
- High latency.
- Packet loss.
- CRC corruption.
- HMAC corruption.
- Replay attack.
- Out-of-order packet.

### 2.8. Acceptance tests co the viet

Co the viet smoke/regression tests cho:

- Normal MA flow van chay.
- Position report flow duoc tao va accept.
- RED failure failover sang BLUE.
- Packet het TTL bi reject.
- Replay packet bi reject.
- CRC/HMAC sai bi reject.
- OPC UA-like loss khong lam ATP trip neu RaSTA vital van OK.
- Vital channel loss lam ATP fail-safe.

## 3. Phan khong chac lam duoc ngay hoac can quyet dinh them

Nhung muc sau co the lam duoc ve mat ky thuat, nhung can can nhac vi co the lam thay doi lon cau truc simulator.

### 3.1. Mo phong RaSTA dung chuan thuc te

RaSTA that co state machine, redundancy management, sequence supervision, retransmission, timing window va safety code rat nghiem ngat. Neu lam "RaSTA-like" thi kha thi. Neu lam "RaSTA chuan cong nghiep" thi khong nen cam ket trong pham vi hien tai.

Khuyen nghi:

- Lam RaSTA-like de day hoc/mo phong.
- Ghi ro khong phai implementation RaSTA certified.

### 3.2. Bao mat HMAC/key management that

HMAC-SHA256 co the lam bang Python standard library. Nhung key lifecycle that gom key rotation, secure storage, certificate, provisioning, audit thi chua co.

Khuyen nghi:

- Dung key gia lap theo `key_id`.
- Ho tro HMAC validation.
- Chua mo phong PKI/secure key storage that.

### 3.3. Radio propagation model thuc te

Mo phong RAP coverage theo `start_m/end_m`, edge degradation va handover la kha thi. Nhung mo phong radio that nhu fading, interference, RSSI, cell load, antenna, BER, multi-path thi qua lon.

Khuyen nghi:

- Lam coverage theo khoang vi tri.
- Them edge factor lam tang jitter/loss.
- Chua lam RF physics chi tiet.

### 3.4. OPC UA that

OPC UA that co session, subscription, node model, security policy, certificate, encoding. Trong simulator nen lam OPC UA-like message bus, khong nen nhung OPC UA stack that neu khong can.

Khuyen nghi:

- Tao `OpcUaSupervisionFrame` voi request/response, retry, timeout, encrypted flag.
- Khong dung no de cap MA/EOA.

### 3.5. Anh huong lon den runtime hien tai

Neu thay `SafeMovementPacket` cu bang `VitalSafePacket` ngay lap tuc, co nguy co pha ATP/ZC/runtime dang chay.

Khuyen nghi:

- Lam lop adapter.
- Giai doan 1: ZC van tao payload cu, boc vao VitalSafePacket.
- Giai doan 2: Onboard CC validate frame, sau do moi convert thanh packet noi bo cho train.
- Giai doan 3: moi them position report va OPC UA-like.

## 4. Kien truc de xuat

Kien truc nen chia thanh 5 lop:

```text
Application logic:
  ATS, ZC, Train CC, ATP, ATO

Message schema:
  MovementAuthorityMessage, PositionReportMessage, TrainStatusMessage

Protocol layer:
  RaSTA-like vital protocol
  OPC UA-like supervision protocol

DCS transport:
  Red/Blue path, RAP coverage, latency, jitter, loss, failover

Dataflow/logging:
  Packet event table, rejection reason, network health
```

Nguyen tac:

- ZC tinh MA/EOA.
- ATP giam sat an toan.
- ATO chi lai tau ben duoi ATP.
- DCS chi van chuyen packet.
- RaSTA-like bao ve ban tin vital.
- OPC UA-like chi dung cho giam sat/lenh van hanh khong vital.

## 5. Giai thich thuat ngu va khai niem

### 5.1. DCS la gi?

DCS la Data Communication System. Trong CBTC, DCS la he thong truyen du lieu giua wayside va tau. No co the gom mang cap quang, switch, radio, access point, antenna va thiet bi tren tau.

Trong mo phong nay, DCS nen duoc hieu la "duong ong van chuyen ban tin". DCS khong duoc tu tinh MA, khong duoc tinh phanh, khong duoc ra lenh ATO. No chi lam cac viec:

- Nhan packet tu nguon.
- Chon duong truyen.
- Gan latency/jitter/loss.
- Kiem tra coverage.
- Bao cao trang thai network.

### 5.2. Red/Blue redundant network la gi?

Red/Blue la hai duong truyen doc lap. Neu duong RED loi, he thong chuyen sang BLUE. Neu BLUE loi, chuyen sang RED. Muc tieu la tranh mot loi don le lam mat vital communication.

Trang thai co ban:

- `OK`: duong truyen tot.
- `DEGRADED`: con truyen duoc nhung cham/mat goi nhieu.
- `LOST`: khong truyen duoc.

### 5.3. Latency, jitter, packet loss, timeout

- Latency: thoi gian goi tin di tu nguon den dich.
- Jitter: do dao dong cua latency. Vi du goi nay 50 ms, goi sau 130 ms.
- Packet loss: goi tin bi mat, ben nhan khong nhan duoc.
- Timeout: qua thoi gian cho phep ma khong co packet hop le.

Trong CBTC, timeout vital communication la nghiem trong. Neu tau khong nhan safe packet hop le trong thoi gian cho phep, ATP phai chuyen sang fail-safe braking.

### 5.4. RAP va coverage

RAP la Radio Access Point. Moi RAP phu song mot doan duong ray:

```text
RAP_01: 0m - 900m
RAP_02: 800m - 1700m
```

Vung giao nhau 800m - 900m giup handover muot hon. Khi tau di tu RAP_01 sang RAP_02, he thong tao handover event.

Gan mep coverage, tin hieu thuong kem hon. Mo phong co the tang:

- Latency.
- Jitter.
- Packet loss.

Ngoai coverage thi vital packet bi mat.

### 5.5. Packet, frame va message khac nhau the nao?

- Message: noi dung nghiep vu, vi du `MA_UPDATE` hoac `POSITION_REPORT`.
- Packet/frame: vo boc truyen tin bao quanh message, gom header, payload va safety data.
- Payload: phan du lieu ben trong packet.

Vi du:

```text
Frame:
  Header: source, destination, sequence, timestamp
  Payload: EOA=1200m, speed_limit=80km/h
  Safety: CRC, HMAC, packet_uuid
```

### 5.6. CRC la gi?

CRC la Cyclic Redundancy Check. No la ma kiem tra loi du lieu. Ben gui tinh CRC tu payload. Ben nhan tinh lai CRC. Neu hai gia tri khac nhau thi du lieu co the da bi sai hoac bi sua.

CRC phat hien loi truyen tin tot, nhung khong phai bao mat. Ke tan cong co the sua payload va tinh lai CRC neu khong co HMAC.

### 5.7. HMAC-SHA256 la gi?

HMAC la ma xac thuc thong diep co dung khoa bi mat. SHA256 la ham bam duoc dung ben trong.

HMAC giup tra loi:

- Packet co dung nguon tin cay khong?
- Payload co bi sua sau khi ky khong?

Neu khong co dung secret key, gan nhu khong the tao HMAC hop le.

### 5.8. TTL la gi?

TTL la Time To Live. No la thoi gian song toi da cua packet.

Vi du packet co `ttl_ms=500`. Neu packet den sau 700 ms thi bi reject, du noi dung dung. Ly do: ban tin an toan qua cu khong con dang tin.

### 5.9. Sequence number va anti-replay

Sequence number la so thu tu tang dan cua packet trong mot session.

Dung de phat hien:

- Duplicate packet: goi bi gui lap lai.
- Replay attack: goi cu bi phat lai.
- Out-of-order: goi den sai thu tu.

Trong vital channel, packet replay/out-of-order khong duoc cap nhat MA/EOA.

### 5.10. Session la gi?

Session la phien ket noi logic giua hai ben, vi du `ZC_01` va `TRAIN_03`.

Packet phai dung `session_id`. Neu sai session thi reject. Dieu nay ngan packet cua phien cu hoac tau khac bi ap nham.

### 5.11. OPC UA-like va RaSTA-like khac nhau the nao?

OPC UA-like:

- Dung cho giam sat va van hanh khong vital.
- Co request/response, retry, timeout.
- Co co `encrypted=True` de mo phong TLS-like.
- Khong duoc cap MA/EOA.

RaSTA-like:

- Dung cho ban tin vital.
- Co sequence, timestamp, TTL, CRC, HMAC, anti-replay, watchdog.
- Neu mat kenh vital thi ATP fail-safe.
- Duoc dung cho MA/EOA, speed profile, position report.

## 6. Lo trinh thuc hien de xuat

### Giai doan 1: Nen tang message va Dataflow

- Them message dataclasses.
- Them packet event log.
- Nang Dataflow Monitor thanh bang packet-level.
- Chua thay doi logic ATP/ZC sau.

Rui ro thap.

### Giai doan 2: VitalSafePacket adapter

- Boc `SafeMovementPacket` hien tai vao `VitalSafePacket`.
- Onboard CC validate TTL/CRC/HMAC/sequence.
- Reject thi khong cap nhat EOA.
- Ghi reject reason.

Rui ro trung binh.

### Giai doan 3: DCS Red/Blue va RAP

- Them `DcsTransport`.
- Them Red/Blue path.
- Them RAP coverage theo position.
- Them failover va handover event.

Rui ro trung binh.

### Giai doan 4: OPC UA-like supervision

- Them supervisory message bus.
- ATS command khong vital di qua OPC UA-like.
- Train status/door/fault/network health di qua OPC UA-like.

Rui ro trung binh.

### Giai doan 5: Fault injection va acceptance tests

- Them cac fault network/protocol.
- Them tests cho TTL, replay, CRC, HMAC, failover, vital loss.

Rui ro thap den trung binh.

## 7. Quyet dinh can chot truoc khi code

Can chot cac diem sau:

- Muon lam "RaSTA-like de mo phong" hay co y dinh bam sat RaSTA chuan that?
- Co can OPC UA stack that khong, hay chi can OPC UA-like?
- RAP coverage cau hinh trong YAML hay hard-code default?
- Vital loss timeout giu `DCS_TIMEOUT_S=1.0` hay tach timeout rieng cho RaSTA session?
- Dataflow Monitor can hien thi bao nhieu dong gan nhat?
- Fault injection dat o GUI, scenario YAML, hay ca hai?

## 8. Ket luan

Phan lon yeu cau co the thuc hien duoc neu lam theo huong mo phong co cau truc, khong co gang dat chuan certified. Cach an toan nhat la khong thay logic ZC/ATP hien co ngay lap tuc, ma them tang protocol/transport bao quanh packet hien tai, sau do mo rong dan.

Phan nen lam truoc:

- Message schemas.
- VitalSafePacket validator.
- Dataflow packet table.
- DCS Red/Blue + RAP coverage.
- Fault injection va tests.

Phan khong nen cam ket la "RaSTA/OPC UA chuan cong nghiep day du", vi do la pham vi rat lon va can specification/certification rieng.
