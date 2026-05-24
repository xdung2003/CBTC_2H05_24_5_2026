# CBTC/ATC Integrated Operations Simulator

Đây là phần mềm mô phỏng vận hành và giám sát tín hiệu metro theo hướng CBTC/ATC, viết bằng Python và Tkinter. Ứng dụng mô phỏng cách trung tâm điều hành, liên khóa, vùng điều khiển, thiết bị ATP/ATO trên tàu, truyền thông DCS và động lực học đoàn tàu phối hợp trong một tuyến metro dùng nguyên lý phân khu di động.

Mục tiêu của dự án là học tập, phân tích và kiểm thử logic mô phỏng. Đây không phải mô hình chứng nhận an toàn, không thay thế thiết kế tín hiệu thực tế và không phải mô hình động lực học đầy đủ của đoàn tàu.

Tài liệu nên đọc kèm:

- [docs/ATC_SYSTEM_REQUIREMENTS_SPEC.md](docs/ATC_SYSTEM_REQUIREMENTS_SPEC.md)
- [docs/ATC_SIMULATION_PARAMETER_BASELINE.md](docs/ATC_SIMULATION_PARAMETER_BASELINE.md)
- [docs/ATC_VERIFICATION_MATRIX.md](docs/ATC_VERIFICATION_MATRIX.md)

## Tính Năng Hiện Tại

- Tính giới hạn cho phép di chuyển theo đoàn tàu phía trước, vùng bảo vệ đuôi tàu, khoảng an toàn và phần chồng lấn bảo vệ.
- Giám sát ATP theo điểm kết thúc quyền chạy, giới hạn tốc độ cố định/tạm thời, độ dốc, sai số vị trí, thời gian phản ứng và thời gian xây dựng lực phanh.
- Điều khiển ATO nằm dưới giới hạn ATP, gồm tiếp cận ga, căn dừng, thời gian dừng đón khách, cho phép mở cửa và căn chỉnh chính xác khi gần điểm dừng.
- Hỗ trợ các chế độ vận hành tự động, lái giới hạn và chạy hạn chế 25 km/h; khi mất truyền thông, ATO bị hạ cấp về chế độ phục hồi an toàn.
- Theo dõi truyền thông DCS, gói quyền chạy an toàn, thứ tự gói tin và cơ chế quá thời gian chờ an toàn.
- Điều phối tàu xuất phát ở đầu tuyến, sức chứa ga, nhiều đường đón tàu trong ga, khóa đường và ưu tiên đường chính.
- Màn hình tổng quan ATS để xem tuyến, ga, nguồn tàu, giới hạn tốc độ, điều kiện tuyến, vị trí tàu, khoảng cách và trạng thái giữ tàu.
- Sửa hạ tầng trên giao diện: thêm/sửa/xóa ga, nguồn tàu, đoạn tốc độ, đoạn dốc, chiều dài tuyến và điều kiện tuyến.
- Hoàn tác/làm lại cho các thay đổi hạ tầng.
- Các chế độ điều tiết giãn cách: tắt điều tiết, giãn cách cố định, theo thời khóa biểu và thích nghi theo điều kiện tuyến.
- Đồng hồ thời gian thực chuẩn Việt Nam trên phần đầu giao diện.
- Chế độ Monte Carlo để chạy nhiều kịch bản ngẫu nhiên và tổng hợp xác suất/KPI.
- Lưu kịch bản YAML và xuất báo cáo vận hành ra thư mục [reports](reports), gồm JSON và các bảng CSV phụ.

## Cấu Trúc Mã Nguồn

- [CBTC_SIM/main_gui.py](CBTC_SIM/main_gui.py): ứng dụng Tkinter, vòng lặp mô phỏng, logic tàu, ATP/ATO, định tuyến ga và hiển thị ATS.
- [CBTC_SIM/core_engine.py](CBTC_SIM/core_engine.py): các thành phần lõi như tính quyền chạy, giám sát DCS, điều khiển trên tàu và mô hình phanh an toàn.
- [CBTC_SIM/physics.py](CBTC_SIM/physics.py): đổi đơn vị, tính khoảng cách phanh, lực kéo, lực cản, khối lượng tương đương và giới hạn giật.
- [CBTC_SIM/config.py](CBTC_SIM/config.py): các tham số nền như bước thời gian, lực phanh, khối lượng và biên an toàn.
- [CBTC_SIM/scenario_loader.py](CBTC_SIM/scenario_loader.py): đọc, chuẩn hóa và lưu kịch bản YAML.
- [CBTC_SIM/headway_manager.py](CBTC_SIM/headway_manager.py): điều tiết xuất phát và ghi nhận giãn cách thực tế giữa các đoàn tàu.
- [CBTC_SIM/monte_carlo.py](CBTC_SIM/monte_carlo.py): chạy Monte Carlo không cần giao diện và tổng hợp KPI xác suất.
- [CBTC_SIM/reporting.py](CBTC_SIM/reporting.py): tạo báo cáo JSON và các bảng CSV.
- Các file `*_regression_test.py`: kiểm thử hồi quy cho vật lý, ATP/ATO, lỗi, ga, giãn cách, Monte Carlo và báo cáo.

## Cách Chạy

Yêu cầu:

- Python 3.11 hoặc tương đương.
- Tkinter.
- Thư viện trong [requirements.txt](requirements.txt), hiện tại chủ yếu là `PyYAML`.

Cài thư viện:

```powershell
pip install -r requirements.txt
```

Chạy giao diện từ thư mục gốc:

```powershell
python CBTC_SIM/main_gui.py
```

Hoặc:

```powershell
cd CBTC_SIM
python main_gui.py
```

Chạy kiểm thử:

```powershell
python CBTC_SIM/smoke_test.py
python CBTC_SIM/physics_regression_test.py
python CBTC_SIM/atp_brake_curve_regression_test.py
python CBTC_SIM/ato_profile_regression_test.py
python CBTC_SIM/fault_regression_test.py
python CBTC_SIM/station_regression_test.py
python CBTC_SIM/headway_regression_test.py
python CBTC_SIM/monte_carlo_regression_test.py
python CBTC_SIM/analytics_report_regression_test.py
```

Bộ kiểm thử hiện là các tập lệnh độc lập, chưa dùng pytest. Khi sửa logic mô phỏng, tối thiểu nên chạy kiểm thử khói và các kiểm thử liên quan trực tiếp. Khi chuẩn bị phát hành hoặc thay đổi hành vi rộng, nên chạy toàn bộ danh sách trên.

## Giao Diện Chính

Khi mở ứng dụng, phần mềm tự tải kịch bản mặc định [CBTC_SIM/default_scenario.yaml](CBTC_SIM/default_scenario.yaml). Giao diện được bố trí như một bàn làm việc kỹ thuật:

- phía trên là thanh điều khiển;
- chính giữa là sơ đồ tuyến ATS 2D;
- bên phải là vùng thông tin và giám sát;
- phía dưới là các bảng tàu;
- cuối cửa sổ là thanh trạng thái.

Thanh điều khiển có các nhóm chính:

- điều khiển mô phỏng: bắt đầu, tạm dừng, khởi động lại và chọn tốc độ mô phỏng `x1`, `x2`, `x5`, `x10`;
- chỉnh sửa phần tử hạ tầng: thêm, xóa, hoàn tác và làm lại;
- đọc/ghi kịch bản: tải YAML, lưu YAML và xuất báo cáo;
- lỗi cho tàu đang chọn: mất DCS và xóa lỗi.

Thanh giãn cách/phân khu cho phép chọn chế độ điều tiết, nhập thông số giãn cách và áp dụng lại mô phỏng. Nút Monte Carlo nằm cố định ở đầu giao diện; khi bật, phần mềm ẩn màn hình mô phỏng thường và chuyển sang bảng thống kê Monte Carlo.

Thanh trạng thái phía dưới hiển thị trạng thái chạy/dừng/lỗi, thời gian mô phỏng, kịch bản đang nạp, số tàu, số tàu đang chạy, tình trạng DCS, số tàu đang phanh khẩn và số va chạm.

## Chế Độ Mô Phỏng Thường

Chế độ thường là không gian vận hành tương tác mặc định. Chế độ này chạy một mô phỏng duy nhất từ kịch bản hiện tại, cập nhật giao diện theo từng bước và cho phép người dùng sửa hạ tầng, gây lỗi, xem giám sát thời gian thực và xuất báo cáo.

| Chức năng | Cơ chế | Kết quả |
| --- | --- | --- |
| Bắt đầu | Chạy vòng lặp mô phỏng. | Thời gian mô phỏng tăng theo bước 0,1 giây. |
| Tạm dừng | Dừng bước mô phỏng nhưng giữ nguyên trạng thái. | Có thể tiếp tục mà không cần khởi động lại. |
| Khởi động lại | Tạo lại mô phỏng từ kịch bản hiện tại. | Xóa trạng thái chạy, thống kê tạm thời và lịch sử hiển thị. |
| Tăng tốc mô phỏng | Chạy nhiều bước mô phỏng trong một lần cập nhật giao diện. | Tối đa `x10` để bảng và đường cong vẫn dễ đọc. |
| Tải kịch bản | Đọc và chuẩn hóa YAML. | Giao diện thông số được đồng bộ lại. |
| Lưu kịch bản | Ghi cấu hình hạ tầng/kịch bản hiện tại. | Không ghi đầy đủ dữ liệu vận hành phát sinh trong lúc chạy. |
| Xuất báo cáo | Chụp trạng thái mô phỏng hiện tại. | Tạo JSON và CSV trong thư mục báo cáo. |
| Mất DCS | Gây lỗi truyền thông cho tàu đang chọn. | Gói quyền chạy bị mất; nếu quá thời gian chờ, tàu chuyển về trạng thái an toàn. |
| Xóa lỗi | Xóa các lỗi ATP/ATO/DCS của tàu đang chọn. | Tàu có thể phục hồi nếu điều kiện an toàn đã hợp lệ. |

Khác với Monte Carlo, chế độ thường giữ đầy đủ hiển thị, nhật ký sự kiện, lịch sử bảng tàu và dấu vết kỹ thuật để phục vụ quan sát chi tiết.

## Tổng Quan ATS Và Sửa Hạ Tầng

Sơ đồ ATS hiển thị:

- tuyến chính, ga và các đường trong ga;
- vùng sinh tàu ở đầu tuyến;
- đoạn tốc độ cố định, đoạn dốc và vùng tốc độ tạm thời;
- điều kiện tuyến;
- thân tàu, mã tàu, trạng thái giữ tàu và khoảng cách tới tàu trước.

Tương tác chính:

- bấm một lần để chọn phần tử;
- bấm đúp để mở hộp thoại sửa;
- dùng nút thêm/xóa để thay đổi hạ tầng;
- hoàn tác/làm lại các thay đổi có thể chỉnh sửa.

Các loại phần tử có thể thêm/sửa gồm ga, đoạn tốc độ, đoạn dốc, chiều dài tuyến, điều kiện tuyến và nguồn tàu. Khi kéo dài tuyến, phần mềm kiểm tra đoạn chưa có giới hạn tốc độ và nhắc thêm giới hạn trước khi nhắc thêm ga mới.

## Điều Tiết Giãn Cách Và Thống Kê

Giãn cách trong kịch bản hoặc trên thanh công cụ là mục tiêu vận hành, không phải kết quả đo. Kết quả thực tế chỉ được ghi khi đoàn tàu thật sự đi qua cổng xuất phát.

| Chế độ | Ý nghĩa | Khi nào giữ tàu |
| --- | --- | --- |
| Tắt điều tiết | Không giữ tàu theo giãn cách kế hoạch. | Chỉ còn các điều kiện nguồn tàu, đường chạy và an toàn. |
| Cố định | Các đoàn tàu được xuất phát theo một khoảng thời gian mục tiêu. | Chưa đến thời điểm kế hoạch hoặc chưa đủ khoảng từ lần xuất phát trước. |
| Thời khóa biểu | Tàu xuất phát theo danh sách thời điểm định trước. | Chưa đến giờ trong thời khóa biểu. |
| Thích nghi | Giống cố định, nhưng có thể tăng giãn cách khi có vùng tốc độ tạm thời và kiểm tra thêm khoảng cách tới tàu trước. | Chưa đủ thời gian, chưa đủ khoảng cách hoặc đang chịu ảnh hưởng của vùng hạn chế. |

Bộ điều tiết giãn cách chỉ quyết định giữ hay thả tàu tại cổng xuất phát. Nó không cấp quyền chạy, không tính điểm kết thúc quyền chạy và không vượt quyền ATP.

Trên giao diện, cấu hình vận hành được gom về một ô `Operation Mode`, không còn ô `Block view` riêng. Mỗi mode tự chọn cơ chế cấp quyền chạy và chỉ hiện các tham số cần nhập:

| Mode trên giao diện | Cơ chế MA/EOA | Tham số hiện ra |
| --- | --- | --- |
| `1 Fixed-block` | `fixed_block` | Số block/khu gian (`blocks_per_section`). |
| `2 Headway target` | `moving_block` | Target seconds, ví dụ 80 giây giữa hai lần dispatch. |
| `3 Timetable` | `moving_block` | Nút load lịch trình từ file YAML/YML hoặc MD. |
| `4 Adaptive tph` | `moving_block` | Số lượt tàu/giờ; phần mềm đổi sang headway mục tiêu tương ứng. |

Trong YAML, `headway.mode` và `block_mode` vẫn là hai trường riêng để lưu dữ liệu:

- `headway.mode: fixed` nghĩa là điều tiết thời gian xuất phát theo một khoảng mục tiêu. Đây không phải là bảo đảm cứ đúng khoảng đó có một tàu tới ga.
- `block_mode: moving_block` nghĩa là ZC cấp quyền chạy theo phân khu di động, dùng bảo vệ tàu phía trước.
- `block_mode: fixed_block` nghĩa là ZC cấp quyền chạy theo các phân khu cố định ngoài ga. Đây là mode fixed-block mới.

## Fixed-Block Runtime Mode

Khi `block_mode: fixed_block`, ZC không tính EOA theo đuôi tàu phía trước. Quyền di chuyển được cấp theo trạng thái chiếm dụng của các phân khu cố định:

- Mỗi khu gian ngoài ga được chia đều thành `capacity_baseline.blocks_per_section` block. Người dùng chỉ cần nhập số block/khu gian, không cần nhập chiều dài từng block.
- Vùng ga được loại khỏi danh sách fixed block. Các đoạn trước ga và sau ga là các section riêng; trong ga vẫn dùng logic route, line/platform và dwell.
- Trên ATS, PSR segment luôn là một lớp riêng và luôn hiện để thể hiện giới hạn tốc độ cố định. Fixed block là lớp riêng bên dưới, dùng cho đóng đường cố định và occupancy; không dùng PSR segment làm phân khu.
- Một block được xem là occupied khi thân tàu còn chồng lên block đó. Khi tàu đã vào ga hoàn toàn, block trước ga được giải phóng.
- ZC chỉ cấp MA tới cuối block hiện tại hoặc thêm một block kế tiếp nếu block đó đang free/reserved hợp lệ. Nếu block kế tiếp bị chiếm, EOA nằm trước vạch đầu block đó một `STOP_SVL_OFFSET_M`.
- Nếu tàu đã hết dwell trong ga nhưng block ngoài ga chưa giải phóng, tàu vẫn bị giữ tại cửa ra ga: EOA = `station_end - STOP_SVL_OFFSET_M`. Chỉ khi block ngoài ga free thì tàu mới được cấp quyền rời ga.
- Các sự kiện EOA trong log sẽ ghi reason `FIXED_BLOCK` khi giới hạn đến từ phân khu cố định.

Trong mô phỏng, occupancy phân khu được suy ra từ vị trí vật lý tàu để đóng vai trò cảm biến phân khu/track circuit. Phần cấp EOA fixed-block không dùng khoảng cách tới đuôi tàu trước như moving-block.

Thống kê giãn cách gồm:

- giãn cách thực tế giữa các cặp tàu xuất phát liên tiếp;
- cặp tàu trước/sau và thời điểm xuất phát của từng tàu;
- giãn cách trung bình đã đo xong;
- khoảng đang mở từ lần xuất phát gần nhất đến thời điểm hiện tại.

Ngoài giãn cách ở đầu tuyến, phần mềm còn tính giãn cách theo từng ga từ góc nhìn hành khách: thời điểm tàu đến ga, thời điểm rời ga, thời gian dừng đón khách, giãn cách đến ga và trung bình theo ga. Thời gian dừng đón khách được giữ tối thiểu 25 giây, kể cả khi kịch bản khai báo thấp hơn.

## Chế Độ Vận Hành ATP, ATO Và DCS

Tàu có ba chế độ vận hành chính:

| Chế độ | Mục đích | Nguyên tắc an toàn |
| --- | --- | --- |
| Tự động ATO | Tự tính tốc độ mục tiêu, lực kéo/phanh, tiếp cận ga, căn dừng, dừng đón khách và mở cửa. | Luôn nằm dưới giới hạn ATP. |
| Lái giới hạn | Mô phỏng chạy thủ công với giới hạn tốc độ và vẫn chịu giám sát ATP. | ATP vẫn có thể cảnh báo, phanh thường hoặc phanh khẩn. |
| Chạy hạn chế 25 km/h | Dùng cho phục hồi hoặc trạng thái truyền thông suy giảm. | Bị giới hạn nghiêm ngặt và có thể trip nếu vi phạm điều kiện an toàn. |

Các chuyển trạng thái quan trọng:

- mất DCS trong chế độ tự động sẽ ức chế ATO và chuyển tàu sang phục hồi hạn chế;
- lỗi ATO khi tàu đang yêu cầu tự động sẽ hạ cấp về lái giới hạn, nhưng ATP vẫn hoạt động;
- xóa lỗi chỉ phục hồi về chế độ yêu cầu khi lỗi đã hết và gói DCS hợp lệ;
- lỗi ATP, dừng khẩn hoặc dừng tức thời có thể giữ tàu trong trạng thái trip/phanh khẩn cho đến khi được xử lý.

ATP tính các đường giám sát như tốc độ cho phép, cảnh báo, phanh thường và phanh khẩn từ điểm kết thúc quyền chạy, giới hạn tốc độ, độ dốc, sai số vị trí và mô hình phanh. ATO hoặc lái thủ công chỉ được sinh lệnh vận hành nằm dưới các giới hạn đó.

## Giám Sát, Nhật Ký Và Bảng Biểu

Các kênh giám sát trong chế độ thường:

| Kênh | Nội dung chính |
| --- | --- |
| Sơ đồ ATS | Tuyến, ga, nguồn tàu, giới hạn tốc độ, điều kiện tuyến, vị trí tàu và trạng thái giữ tàu. |
| Bảng từng tàu | Tốc độ, mục tiêu, quyền chạy, các đường phanh, trạng thái ATP/ATO/DCS, cửa, dừng ga và lỗi. |
| Hạ tầng | Trạng thái ga, nguồn tàu, đoạn tuyến, sức chứa ga và thông tin đường trong ga. |
| Kỹ thuật | Biểu đồ thời gian - vị trí, thông tin phanh, giám sát ATP và dữ liệu vật lý. |
| Phân tích & chẩn đoán | Giãn cách, thống kê hành khách theo ga, năng lượng, số lần phanh, DCS, sai số vị trí, cảnh báo và nhật ký sự kiện. |
| Thanh trạng thái | Tóm tắt nhanh số tàu, tàu đang chạy, DCS, phanh khẩn và va chạm. |

Nhật ký có ba lớp:

- nhật ký chi tiết theo tàu, dùng để xuất bảng sự kiện;
- hàng đợi sự kiện ngắn cho giao diện, dùng để cập nhật bảng chẩn đoán;
- dấu vết kỹ thuật ATP/ZC theo từng bước, dùng để xuất bảng phân tích sâu.

Các sự kiện đang được ghi gồm mất gói DCS, quá thời gian chờ DCS, phục hồi DCS, thay đổi hành động ATP, thay đổi cảnh báo, cho phép mở cửa, trạng thái dừng ga, hoàn thành dừng, căn chỉnh gần điểm dừng, chuyển lỗi và phát hiện va chạm.

## Chế Độ Monte Carlo

Monte Carlo dùng [CBTC_SIM/monte_carlo.py](CBTC_SIM/monte_carlo.py) để chạy nhiều bản sao mô phỏng không cần giao diện từ kịch bản hiện tại. Chế độ này không thay thế mô phỏng thường; nó là lớp chạy hàng loạt để trả lời các câu hỏi xác suất.

Mỗi mẫu sẽ sao chép kịch bản gốc, lấy ngẫu nhiên một số tham số, áp tạm thời vào mô phỏng, chạy đến giới hạn thời gian rồi gom KPI. Để chạy nhanh 100 hoặc 1000 mẫu, chế độ này tắt dấu vết chi tiết, tắt nhật ký sự kiện nặng và chỉ tính thống kê chi tiết ở cuối mỗi mẫu. Sau mỗi mẫu, các tham số toàn cục được khôi phục để không ảnh hưởng chế độ thường.

Các nhóm tham số được lấy ngẫu nhiên:

- truyền thông DCS: độ trễ gói tin, xác suất mất gói và thời gian mất liên lạc;
- định vị/cảm biến: trôi odometry, nhiễu vị trí và sai số căn dừng;
- động học ATP/ATO: thời gian phản ứng, thời gian xây dựng lực phanh, hệ số phanh, bám dính và giới hạn giật;
- vận hành: giãn cách mục tiêu, biến thiên thời gian dừng, dao động thời điểm xuất phát và trễ giải phóng đường;
- hạ tầng: vùng tốc độ tạm thời và tốc độ giải phóng khi tiếp cận ga.

Bảng phân phối mặc định được viết theo ngôn ngữ vận hành:

| Nhóm | Phân phối xác suất | Dải mặc định |
| --- | --- | --- |
| Giãn cách mục tiêu | Đều | 120 đến 300 giây |
| Dao động thời điểm xuất phát | Đều | 1 đến 3 giây |
| Thời gian dừng ở ga | Thời gian dừng gốc cộng nhiễu chuẩn | Độ lệch chuẩn 5 giây, chặn trong 25 đến 90 giây |
| Độ trễ DCS | Đều | 0,10 đến 0,50 giây |
| Xác suất mất gói DCS | Cố định | 0,5% |
| Thời gian mất liên lạc DCS | Đều | 2 đến 5 giây |
| Trôi định vị | Tam giác | thấp 1%, thường gặp 2%, cao 3% |
| Nhiễu vị trí | Chuẩn có chặn | trung bình 2,0 m, độ lệch chuẩn 0,5 m, chặn 0,2 đến 6,0 m |
| Sai số căn dừng | Trị tuyệt đối của nhiễu chuẩn | độ lệch chuẩn 0,5 m |
| Thời gian phản ứng | Chuẩn có chặn | trung bình 2,0 giây, độ lệch chuẩn 0,4 giây, chặn 0,5 đến 4,0 giây |
| Thời gian xây dựng lực phanh | Đều | 1 đến 2 giây |
| Hệ số phanh | Cố định | 1,2 |
| Bám dính | Tam giác | thấp 0,55, thường gặp 0,70, cao 1,00 |
| Giới hạn giật | Đều | 0,8 đến 1,0 |
| Trễ giải phóng đường | Cố định | 180 giây |
| Tốc độ giải phóng khi tiếp cận ga | Đều | 8 đến 15 km/h |
| Có vùng tốc độ tạm thời | Bernoulli | xác suất 25% |
| Tốc độ vùng tạm thời | Chọn rời rạc | 15, 25 hoặc 40 km/h |
| Vị trí vùng tạm thời | Đều theo chiều dài tuyến | bắt đầu trong nửa đầu tuyến, chiều dài thêm 200 đến 700 m |

Bảng Monte Carlo hiển thị tiến độ, pha đang chạy, phân phối giãn cách, dịch vụ hành khách theo ga, xác suất phanh/cảnh báo/va chạm, năng lực thông qua, năng lượng kéo và các mẫu gần nhất.

## Định Tuyến Ga, Sức Chứa Và Khóa Đường

Một ga có thể có nhiều đường đón tàu. Đường số 0 là đường chính; các đường còn lại là đường phụ hoặc platform. Khi tàu cần vào ga, hệ thống tìm đường khả dụng theo thứ tự từ thấp đến cao, ưu tiên đường chính trước.

Quy tắc chính:

1. Nếu tàu chưa có đường vào ga, hệ thống tìm đường còn trống theo thứ tự.
2. Sau khi một đường đã được cấp, tàu không tự động đổi sang đường khác.
3. Nếu đường đã cấp không còn an toàn hoặc không còn khả dụng, tàu bị giữ ngoài ga thay vì tự đổi đường.
4. Khi tàu vào đường đã cấp, đường đó chuyển sang trạng thái bị khóa/chiếm dụng.
5. Đường chỉ được giải phóng khi đuôi tàu đã qua khỏi vùng ga và biên giải phóng.

Ga cuối được nhận diện theo cả cuối tuyến vật lý và điểm dừng cuối trong lịch trình. Vì vậy nếu lịch trình chỉ yêu cầu tàu chạy đến một ga giữa tuyến, ga đó vẫn được xem như ga cuối của hành trình và tàu có thể giữ dừng vô hạn tại đó.

## Bảng Tàu

Mỗi tàu có một bảng trong tab `Trains`, gồm:

- tốc độ thực, tốc độ cho phép, đường cảnh báo, phanh thường và phanh khẩn;
- khoảng cách đến mục tiêu, điểm kết thúc quyền chạy và cảnh báo ATP;
- chế độ vận hành;
- trạng thái cửa, căn dừng, giữ tàu, dừng ga và căn chỉnh gần điểm dừng;
- biểu đồ tốc độ và đường phanh;
- nút dừng khẩn/cho chạy lại tùy trạng thái;
- nút xem chi tiết về tốc độ, mục tiêu, định vị, DCS, cửa, phanh, ga và lịch dừng.

Tính năng căn chỉnh chính xác gần điểm dừng có trong logic mô phỏng, nhưng bảng hiện tại không hiển thị nút riêng. Lệnh căn chỉnh chỉ được chấp nhận khi tàu đã đứng yên và còn nằm trong cửa sổ khoảng cách hợp lệ.

## Kịch Bản YAML

Kịch bản được đọc bằng [CBTC_SIM/scenario_loader.py](CBTC_SIM/scenario_loader.py). File mặc định là [CBTC_SIM/default_scenario.yaml](CBTC_SIM/default_scenario.yaml).

Các nhóm cấu hình chính gồm tên kịch bản, hiển thị, mặc định của tàu, đoạn tuyến, ga dừng, tàu ban đầu, nguồn tàu, điều kiện tuyến, điều tiết giãn cách và tham số phân khu cố định. Tên khóa YAML vẫn giữ bằng tiếng Anh vì đây là định dạng dữ liệu mà chương trình đọc trực tiếp.

Ví dụ tối thiểu:

```yaml
name: Example

display:
  window_title: CBTC ATC Simulation - Example
  track_min_m: -220
  track_max_m: 1200
  track_labels: [0, 500, 1000, 1200]

train_defaults:
  length_m: 60
  mass_kg: 291600
  drive_mode: ATO
  max_manual_speed_kmh: 45

track:
  segments:
    - start_m: 0
      end_m: 1200
      gradient: 0.0
      psr_kmh: 80

scheduled_stops:
  - name: STATION_1000
    pos_m: 1000
    dwell_s: 30
    length_m: 160
    capacity: 3

trains:
  - id: T1
    start_pos: 0
    drive_mode: ATO
    color: "#1f77b4"

source_trains:
  - name: SRC
    start_m: -200
    length_m: 200
    capacity: 3
    total_trains: 3

line_conditions:
  - start_m: 300
    end_m: 700
    condition: wet

block_mode: fixed_block

headway:
  mode: fixed
  target_headway_s: 120

capacity_baseline:
  blocks_per_section: 4
```

Khi nạp kịch bản, phần mềm sắp xếp đoạn tuyến và ga theo vị trí, đặt mặc định hợp lý cho các trường thiếu, áp dụng chế độ lái mặc định cho tàu chưa khai báo, giữ điều kiện tuyến để hiển thị trên ATS và chuẩn hóa chế độ giãn cách. Nguồn tàu khi chạy hiện dùng vùng cố định từ -200 m đến 0 m; sức chứa nguồn đồng thời là số làn và số tàu được sinh cho nguồn đó.

Các bí danh chế độ lái:

- `ATO` hoặc `AUTOMATIC`: tự động;
- `LMD`, `MCS`, `MTC` hoặc `MANUAL`: lái giới hạn;
- `CMD25`, `RM`, `RM25`, `RESTRICTED` hoặc `RESTRICTED_MANUAL`: chạy hạn chế 25 km/h.

## Xuất Báo Cáo

Nút xuất báo cáo gọi [CBTC_SIM/reporting.py](CBTC_SIM/reporting.py) và ghi file vào [reports](reports) với tên dạng:

```text
YYYYMMDD_HHMMSS_<ten_kich_ban>.json
```

Báo cáo JSON gồm:

- thông tin kịch bản và thời điểm mô phỏng;
- thống kê giãn cách, thống kê ga, độ trễ xuất phát, năng lực thông qua, năng lượng, số lần phanh thường/phanh khẩn, số va chạm và thời gian hành trình;
- trạng thái từng tàu: vị trí, tốc độ, quyền chạy, giới hạn tốc độ, ATP/ATO, cửa, định vị, DCS và các đường giám sát;
- đoạn tuyến và vùng tốc độ tạm thời.

Ngoài JSON, phần mềm tạo ba bảng CSV:

| File | Nội dung |
| --- | --- |
| `.trains.csv` | Một dòng cho mỗi tàu tại thời điểm xuất báo cáo, gồm trạng thái vận hành, năng lượng, lực tính toán khi chạy, giãn cách, phanh và va chạm. |
| `.events.csv` | Nhật ký sự kiện theo tàu, gồm thời điểm, lý do, vị trí, tốc độ, trạng thái ATP/ATO và thông tin va chạm nếu có. |
| `.atp_trace.csv` | Dấu vết kỹ thuật theo từng bước, phục vụ phân tích sâu về quyền chạy, sai số vị trí, đường phanh, DCS, lệnh ATO và trạng thái ga. |

Báo cáo hiện không còn xuất bảng so sánh năng lực phân khu di động với phân khu cố định và không còn tạo file `.capacity.csv`.

## Lưu Kịch Bản

Nút lưu YAML ghi lại phần cấu hình có thể chỉnh sửa:

- phạm vi hiển thị;
- mặc định của tàu;
- đoạn tuyến, độ dốc và giới hạn tốc độ;
- ga dừng;
- định nghĩa tàu gốc;
- nguồn tàu;
- điều kiện tuyến.

Vùng tốc độ tạm thời phát sinh trong lúc chạy được giữ trong hoàn tác và báo cáo, nhưng không được ghi vào YAML đầu ra của kịch bản.

## Quy Trình Mô Phỏng Tóm Tắt

Mỗi bước mô phỏng:

1. Cập nhật nguồn tàu, tuyến vào ga và vùng bảo vệ.
2. Vùng điều khiển tính quyền chạy cho từng tàu.
3. DCS tạo gói quyền chạy an toàn, có thể có độ trễ hoặc quá thời gian chờ.
4. Thiết bị trên tàu nhận gói hợp lệ mới nhất.
5. Tàu cập nhật định vị, hiệu chỉnh beacon/balise và độ bất định vị trí.
6. ATP tính các đường giám sát và quyết định cảnh báo/phanh nếu cần.
7. ATO hoặc chế độ lái giới hạn tính lực kéo/phanh trong giới hạn ATP.
8. Mô hình vật lý cập nhật gia tốc, tốc độ và vị trí với giới hạn giật.
9. Logic ga, dừng đón khách, cửa, căn chỉnh, DCS và an toàn sự cố cập nhật trạng thái và sự kiện.
10. Giao diện cập nhật sơ đồ ATS, bảng tàu và tab phân tích/chẩn đoán.

Những nguyên tắc cốt lõi khi sửa phần mềm:

- quyền chạy chỉ được tính từ trạng thái tuyến, tàu phía trước và biên an toàn;
- DCS chỉ vận chuyển gói quyền chạy, thiết bị trên tàu chỉ dùng gói còn hợp lệ;
- ATP là lớp giám sát an toàn, có quyền cảnh báo, phanh thường và phanh khẩn;
- ATO và lái thủ công không được nâng giới hạn cao hơn ATP;
- đường vào ga đã cấp không được tự ý đổi sau khi đã khóa;
- cửa, dừng ga và căn chỉnh gần điểm dừng chỉ được phép khi đủ điều kiện an toàn;
- va chạm là trạng thái thân tàu chồng lấn thật, khác với cảnh báo vi phạm biên an toàn;
- báo cáo và thống kê chỉ để quan sát, không được dùng để cấp ngược quyền chạy.

## Giới Hạn Hiện Tại

- Nhiều tham số ATP/ATO vẫn là giả định mô phỏng và cần đối chiếu tài liệu khi sửa.
- `main_gui.py` vẫn còn trộn nhiều logic mô phỏng và giao diện.
- Một số bảng giao diện cũ còn tồn tại trong mã để giữ hàm gọi lại, nhưng không hiển thị trong không gian làm việc hiện tại.
- Nguồn tàu trong YAML có khai báo vị trí và chiều dài, nhưng khi chạy hiện vẫn dùng vùng nguồn cố định `-200..0`.
- Mô phỏng ưu tiên hành vi an toàn khi có sự cố và khả năng quan sát hơn độ chính xác vật lý/chứng nhận.

## Hướng Nâng Cấp Theo Bước Nhỏ

1. Giữ kiểm thử hồi quy xanh trước khi thêm tính năng mới.
2. Tách hàm phụ trợ và hằng số nhỏ trước, tránh di chuyển lớp lớn khi chưa có kiểm thử bao phủ.
3. Thay các đoạn bỏ qua lỗi im lặng bằng ghi log có kiểm soát.
4. Chuẩn hóa hợp đồng báo cáo/kịch bản bằng phiên bản lược đồ, hàm tạo thời điểm/tên file và kiểm tra nhẹ.
5. Tách dần lõi mô phỏng khỏi Tkinter sau khi kiểm thử ATP/ATO/ga/tàu ổn định.
6. Cập nhật README, tài liệu và ma trận kiểm chứng khi thay đổi hành vi người dùng hoặc giả định an toàn.

## Nguyên Tắc Khi Sửa Tiếp

Khi thay đổi logic an toàn hoặc vận hành, nên cập nhật đồng thời:

1. mã trong `CBTC_SIM`;
2. README này;
3. tài liệu liên quan trong [docs](docs);
4. ma trận kiểm chứng nếu thay đổi hành vi kiểm thử.
