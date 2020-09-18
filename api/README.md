# AutowareScenarioAPI

## API of scenario_api

（以下、特に記載がない場合、関数の bool 返り値は正常終了を示すものである）

### Autoware start API

#### bool setEgoCarName(const std::string & name)

- 自車の名前を登録する
- isObjectInArea, sendGoalPoint 等の API を使用する際に必要

#### bool waitAutowareInitialize()

- Autoware の準備ができるまで待機する
- 以下の全ての API を使う前にこちらを使用すること

#### bool sendStartPoint(const geometry_msgs::Pose pose, const bool wait_ready = true, const std::string & frame_type = "Center")

- 自車両の/map 座標系でのスタート位置を指定する
- wait_ready は Autoware がスタート位置を受付完了するまで待機するオプションのため、基本的には true として使用すること
- ただし、一度の Autoware 起動において, 自社に対して二度以上 sendStartPoint を使用する場合は二回目以降 false とすること（Autoware 側の仕様が不完全なため.Autoware が修正し次第対応予定）
- frame_typeとしては以下のものが指定可能である
  - "Center" (default) ... 指定した座標にEgoCarのbase_linkが来るようにEgoCarが出現する
  - "Front"          ... 指定した座標にEgoCarのポリゴン後端が来るようにEgoCarが出現する
  - "Rear"           ... 指定した座標にEgoCarのポリゴン前端が来るようにEgoCarが出現する

#### bool sendGoalPoint(const std::string & name, const geometry_msgs::Pose pose, const bool wait_ready = true, const std::string & frame_type = "Center")

- 車両(自車両/NPC)の名前と、車両の/map 座標系でのゴール位置を指定する.
- setEgoCarName を使用した後に使用すること
- sendStartPoint/addNPC を使用したあとに使用すること
- wait_ready は Autoware/Simulator がゴール位置を受付完了するまで待機するオプションのため、基本的には true として使用すること
- ただし、一度の Autoware 起動において, 自車に対して二度以上 sendStartPoint を使用する場合は二回目以降 false とすること（Autoware 側の仕様が不完全なため.Autoware が修正し次第対応予定）
- frame_typeとしては以下のものが指定可能である
  - "Cener" (default) ... 指定した座標にEgoCarのbase_link/NPCのポリゴン中心が来るようにゴールを設置する
  - "Front"          ... 指定した座標にEgoCar/NPCのポリゴン後端が来るようにゴールを設置する
  - "Rear"           ... 指定した座標にEgoCar/NPCのポリゴン前端が来るようにゴールを設置する

#### bool sendCheckPoint(const std::string & name, const geometry_msgs::Pose pose, const bool wait_ready = true, const std::string & frame_type = "Center")

- 車両(自車両/NPC)の名前と、車両の/map 座標系でのゴール位置を指定する. (NPC... not implemented (ignored))
- 車両(自車両/NPC)の/map 座標系でのチェックポイント位置を指定する
- チェックポイントは指定しなくても問題なく、その場合スタート・ゴールを最短で結ぶ経路が内部で生成される
- sendGoalPoint を使用したあとに使用すること
- チェックポイントを指定した順番に通過するような経路を Autoware が内部で生成するが、この際同じ経路を二度通過するようにチェックポイントを指定することは禁止されている
- wait_ready は Autoware がチェックポイント位置を受付完了するまで待機するオプションのため、基本的には true として使用すること
- ただし、一度の Autoware 起動において, 自社に対して二度以上 sendStartPoint を使用する場合は二回目以降 false とすること（Autoware 側の仕様が不完全なため.Autoware が修正し次第対応予定）
- frame_typeとしては以下のものが指定可能である
  - "Center" (default) ... 指定した座標にEgoCarのbase_link/NPCのポリゴン中心が来るようにチェックポイントを設置する
  - "Front"          ... 指定した座標にEgoCar/NPCのポリゴン後端が来るようにチェックポイントを設置する
  - "Rear"           ... 指定した座標にEgoCar/NPCのポリゴン前端が来るようにチェックポイントを設置する

#### bool sendStartVelocity(const double velocity)

- 開始時の初期速度を送る
- （注：重要）
  - Planning Simulatorの動作モード"vehicle_model_type"が、速度を入力としているモードでは使用しないこと（加速度の場合はOK）
    - .ivのVehicle Cmd Gateモジュールの影響で開始直後に大幅に減速するため
  - 自己位置同定をする場合には使用しないこと（E自己位置と速度の不整合によりEKFに不具合が生じるため）
    - Planning Simulatorの場合は問題ない
  - sendStartVelocityからsendEngageまでの時間を長く取らないこと
    - ~5s程度とすること
    - VelocityController内のPIDのI項が溜まって挙動がおかしくなる、といったことが考えられるため

#### bool setMaxSpeed(double velocity)

- 自車の上限速度を送信する
- こちらの上限速度に対して、地図に埋め込まれている上限速度や Autoware 内部での加速度・ジャーク制限等による上限速度が優先されるため注意

#### bool sendEngage(const bool engage)

- 自車とシミュレータのエンゲージを送信する
  - 自車とシミュレータのエンゲージはほぼ同タイミングである必要があるため、一つのAPIとしてまとめている
- sendGoalPoint(,sendCheckPoint)を使用したあとに使用すること

### basic API

#### bool isEgoCarName(const std::string & name)

- setEgoCarName でセットした名前と等しいか否かの比較を行う
- Action や Condition を実装する際、与えられた Actor や Trigger が EgoCar であるかを判別したいときに用いる

#### bool isAPIReady()

- ScenarioAPI が使用可能かどうかを返す
- sendGoalPoint を使用したあとに使用すること

#### bool waitAPIReady()

- ScenarioAPI が使用可能になるまで待機する
- sendGoalPoint を使用したあとに使用すること
- **以下の全ての API を使う前にこちらを使用すること**

#### bool updateState()

- Simulator の状態を更新する(**not implemented**)
- [TODO]Planning Simulator にステップ実行の機能がないため、未実装

### coordinate API

#### bool setFrameId(std::string frame_id, const geometry_msgs::Pose pose)

- FrameId の座標をセットする。Pose は/map 座標系で入力すること
- すでに登録されている FrameId が入力された場合は False を返す

#### geometry_msgs::Pose getRelativePose(std::string frame_id, const geometry_msgs::Pose pose)

- すでに登録済の FrameId からの相対位置を/map 座標系での位置として出力する
- [TODO]未登録の FrameId を入力した場合は全て 0 が入力された Pose が出力されるが、WARNING 出力がされるだけでエラーを吐かないので注意

### basic self vehicle API

#### Pose2D getCurrentPose()

- 自車両の現在位置を/map 座標系で出力する
- Pose2D... struct (Pose2D.x, Pose2D.y, Pose2D.yaw)

#### geometry_msgs::PoseStamped getCurrentPoseRos()

- 自車両の現在位置を/map 座標系で出力する

#### double getVelocity()

- 自車両の現在速度を出力する

#### double getAccel()

- 自車両の現在加速度を出力する
- <TODO>Low Pass Filter 等かけていないので精度が低い可能性あり

#### double getJerk()

- 自車両の現在ジャークを出力する
- <TODO>Low Pass Filter 等かけていないので精度が低い可能性あり

#### double getMoveDistance()

- 自車両の走行移動距離を返す
- 走行移動距離にはΔ-poseから計算した積算距離を用いている

#### bool isStopped(double thresh_velocity = 10e-3)

- 自車が停止しているかどうかを判定する
- 停止判定の閾値速度を変更したい場合は thresh_velocity を入力すること

#### bool isInArea(geometry_msgs::Pose pose, double dist_thresh, double delta_yaw_thresh) [TODO]削除予定（現状の他のモジュールとの整合性を残すため残留）

- 自車が指定された Pose に対して一定距離(dist_thresh)、一定角度(delta_yaw_thresh)以内にいるかどうかを判定する

### additonal self vehicle API

#### bool willLaneChange() (**temporary implemented(it has bug)**)

- 自車がレーンチェンジを実行しようとしているかどうかを返す
- [TODO]Autoware に適切な API が存在しないため、現在はウインカーを出しているかどうかを返す API となっており、右左折時に true となるため注意

#### bool getLeftBlinker()

- 自車が左ウインカーを点灯させているかどうかを返す

#### bool getRightBlinker()

- 自車が右ウインカーを点灯させているかどうかを返す

### lane API

#### bool getCurrentLaneID(int& current_id, double max_dist = 3.0, double max_deleta_yaw = M_PI / 4.0)

- 現在車両（base_link）が存在するレーンを返す
- 現在レーンとしてみなされる許容距離(max_dist)、許容角度差(deleta_yaw)が設定可能

#### bool getDistancefromCenterLine(double& dist_from_center_line)

- 現在いるレーンのセンターラインから車両位置（base_link）までの距離を返す

#### bool isInLane()

- 自車ポリゴンが単一レーン内に存在するかどうかを返す
- 白線踏み越しなどに使用する想定の API

### obstacle API

#### double getMinimumDistanceToObstacle(bool consider_height)

- 全ての障害物との最小距離を返す(衝突時は 0 を返す)
- consider_height は衝突判定時に高さ方向を考慮するかどうかのフラグである
- dummy_perception_publisher の出力する障害物点群は Z 方向が適当であるため,基本的には consider_height=false として使用すること

### NPC API

#### bool addNPC(const std::string& npc_type, const std::string name, geometry_msgs::Pose pose, const double velocity, const bool stop_by_vehicle = false, const std::string & frame_type = "Center")

- 指定された位置・姿勢の場所に指定された速度で NPC を発生させる
- 同一の name を二度以上指定した場合 false が返され NPC は発生しない
- npc_type としては以下のものが指定可能である
  - 歩行者："pedestrian"…指定された場所から直進する動きを行う(レーンチェンジや右左折指令は受け付けない)
  - 自転車："bicycle"…(デフォルト状態では)指定された場所から直近レーンの左端に沿って移動する動きを行う
  - バイク："motorbike"…(デフォルト状態では)指定された場所から直近レーンに沿って移動する動きを行う
  - 自動車："car"…(デフォルト状態では)指定された場所から直近レーンに沿って移動する動きを行う
  - バス："bus"…(デフォルト状態では)指定された場所から直近レーンに沿って移動する動きを行う
  - トラック："truck"…(デフォルト状態では)指定された場所から直近レーンに沿って移動する動きを行う
- stop_by_vehicleをTrueにすると目の前に自車両がいた場合ゆっくりと止まる動きをする
  - NPCの進行方向直線状に自車両がいるときのみ減速し、カーブ時などは減速しないので注意
- frame_typeとしては以下のものが指定可能である
  - "Center" (default) ... 指定した座標にNPCのポリゴン中心が来るようにNPCが出現する
  - "Front"          ... 指定した座標にNPCのポリゴン後端が来るようにNPCが出現する
  - "Rear"           ... 指定した座標にNPCのポリゴン前端が来るようにNPCが出現する

#### bool changeNPCVelocity(const std::string name, double velocity)

- 指定された NPC の速度を変更する
- NPCの速度変化に伴う加速度は以下の通りとなっている
  - 事前にNPCのnameに対してchangeNPCAccelMinが打たれている場合はその加速度で減速する
  - 事前にNPCのnameに対してchangeNPCAccelMaxが打たれてている場合はその加速度で加速する
  - 上記に当てはまらない場合、速度は指定直後に不連続に（加速度+∞または-∞として）変化する
- addNPC で発生させた name 以外を指定した場合は false が返されなにも起きない

#### bool changeNPCAccelMin(const std::string & name, const double accel)

- 指定された NPC の加速度下限を変更する
- 加速度は符号付きで指定すること。基本的にマイナス値のみ指示すること

#### bool changeNPCAccelMax(const std::string & name, const double accel)

- 指定された NPC の加速度上限を変更する
- 加速度は符号付きで指定すること。基本的にプラス値のみ指示すること

#### bool changeNPCVelocityWithAccel(const std::string name, double velocity, double accel)

- 指定された NPC の速度を変更する
- 速度は指定された加速度(絶対値)で変化する
  - 減速時でもaccelには正の値を入れること
- addNPC で発生させた npc_type 以外を指定した場合は false が返されなにも起きない

#### bool changeNPCConsiderVehicle(const std::string & name, const bool consider_ego_vehicle)

- 指定された NPC の動作モードを変更する
- consider_ego_vehicle=trueとすると自車両が前にいる時に減速・停止する
- consider_ego_vehicle=falseとすると自車両に影響されずに動作する

#### bool changeNPCLaneChangeLeft(const std::string name)

- 指定された NPC(npc_type が"car"の NPC のみ有効)に対して左方向へのレーンチェンジ要求を出す
- addNPC で発生させた name 以外を指定した場合は false が返されなにも起きない
- [TODO]指定時に対象となるレーンがなかった場合は true が返されるが何も起きないため注意

#### bool changeNPCLaneChangeRight(const std::string name)

- 指定された NPC(npc_type が"car"の NPC のみ有効)に対して右方向へのレーンチェンジ要求を出す
- addNPC で発生させた name 以外を指定した場合は false が返されなにも起きない
- [TODO]指定時に対象となるレーンがなかった場合は true が返されるが何も起きないため注意

#### bool changeNPCLaneChange(const std::string& name, const int target_lane_id)

- 指定された NPC(npc_type が"car"の NPC のみ有効)に対して指定されたレーン ID へのレーンチェンジ要求を出す
- 現在のレーンと指定された ID のレーンが隣接されている場合にのみレーンチェンジを行う
  - 隣接するレーンに指定されたレーンIDがない場合、レーンチェンジ指令は受け入れずそのまま進行するが、後に隣接レーンが見つかった場合はそのタイミングでレーンチェンジを行う
- addNPC で発生させた name 以外を指定した場合は false が返されなにも起きない
- [TODO]指定時に対象となるレーンがなかった場合は true が返されるが何も起きないため注意

#### bool changeNPCUturn(const std::string& name)

- 指定された NPC(npc_type が"car"の NPC のみ有効)に対して U ターン要求を出す
- addNPC で発生させた name 以外を指定した場合は false が返されなにも起きない
- 指定時に対象となるレーンがなかった場合は true が返されるが何も起きないため注意
- U ターンのため,左右いずれか片方にしか対象がないことを前提としている（そうでない場合対象レーンの方向はランダムに決定される）

#### bool changeNPCTurnLeft(const std::string name)

- 指定された NPC(npc_type が"car"の NPC のみ有効)に対してレーン左折要求を出す
- addNPC で発生させた name 以外を指定した場合は false が返されなにも起きない
- 指定したタイミング以降、常に進行経路の優先順位が左折・直進・右折となる（他指令による上書きは可能）

#### bool changeNPCTurnRight(const std::string name)

- 指定された NPC(npc_type が"car"の NPC のみ有効)に対してレーン右折要求を出す
- addNPC で発生させた name 以外を指定した場合は false が返されなにも起きない
- 指定したタイミング以降、常に進行経路の優先順位が右折・直進・左折となる（他指令による上書きは可能）

#### bool changeNPCNoTurn(const std::string name)

- 指定された NPC(npc_type が"car"の NPC のみ有効)に対してレーン直進要求を出す（デフォルト状態）
- addNPC で発生させた name 以外を指定した場合は false が返されなにも起きない
- 指定したタイミング以降、常に進行経路の優先順位が直進・右左折となる（他指令による上書きは可能）

#### bool changeNPCIgnoreLane(const std::string name)

- 指定された NPC(npc_type が"car"の NPC のみ有効)に対して直進要求を出す（レーンは無視する）
- addNPC で発生させた name 以外を指定した場合は false が返されなにも起きない
- 指定したタイミング以降、常に直進を行う（他指令による上書きは可能）

#### bool deleteNPC(const std::string id)

- 指定された NPC をシミュレーションから削除する
- addNPC で発生させた name 以外を指定した場合は false が返されなにも起きない

#### bool calcDistToNPC(double& dist_to_npc, const std::string id)

- 指定された NPC と自車とのポリゴン上での最短距離を計算する
- addNPC で発生させた name 以外を指定した場合は false が返されなにも起きない

#### bool calcDistToNPCFromNPC(double & distance, const std::string & npc1_name, const std::string & npc2_name)

- 指定されたnpc1とnpc12のポリゴン上での最短距離を計算する
- addNPC で発生させた name 以外を指定した場合は false が返される

#### bool finishNPCLaneChange(const std::string & name, bool * finish_lane_change)

- 指定されたNPCがレーンチェンジを完了したかどうかがfinish_lane_changeに返される
- レーンチェンジが完了している状態や、そもそもレーンチェンジをしようとしていない場合はTrueとなる
- 不正なNPCが指定された場合は関数の返り値としてFalseが返される

#### bool finishNPCVelocityChange(const std::string & name, bool * finish_velocity_change)

- 指定されたNPCが速度変更を完了したかどうかがfinish_lane_changeに返される
- 速度変更が完了している状態や、そもそも速度変更をしようとしていない場合はTrueとなる
- 不正なNPCが指定された場合は関数の返り値としてFalseが返される

#### bool getNPCVelocity(const std::string name, double * velocity)

- 指定されたNPCの速度をvelocityに返す
- 不正なNPCが指定された場合は関数の返り値としてFalseが返される

#### bool getNPCAccel(const std::string name, double * accel)

- 指定されたNPCの加速度をvelocityに返す
- 不正なNPCが指定された場合は関数の返り値としてFalseが返される

### traffic light API

#### bool setTrafficLightColor(int traffic_id, std::string traffic_color, bool use_traffic_light)

- use_traffic_light=false のとき
  - 指定された traffic_relation_id,traffic_color("Red", "Yellow", "Green")としての検出結果を autoware に対して発行する
  - traffic_relation_id に紐づく信号機に対して全ての検出結果が出力される
  - id は type:regulatory_element, subtype:traffic_light の relation id を指定すること
  - 同一の id に対する信号色指定が複数行われた場合、最後に指定された色のみ有効となる
- use_traffic_light=true のとき (**not implemented**)
  - シミュレータ内での信号機オブジェクトの色を変更する(未実装)

#### bool setTrafficLightArrow(const int traffic_id, const std::string traffic_arrow, const bool use_traffic_light)

- use_traffic_light=false のとき
  - 指定された traffic_relation_id,traffic_arrow("Left", "Right", "Up")としての検出結果を autoware に対して発行する
  - traffic_relation_id に紐づく信号機に対して全ての検出結果が出力される
  - id は type:regulatory_element, subtype:traffic_light の relation id を指定すること
  - 同一の id に対する矢印指定が複数回行われた場合、全ての矢印が有効となる
- use_traffic_light=true のとき (**not implemented**)
  - シミュレータ内での信号機オブジェクトの矢印を変更する(未実装)


#### bool resetTrafficLightColor(const int traffic_id, const bool use_traffic_light)

- use_traffic_light=false のとき
  - 指定された traffic_relation_idの色をリセットする（矢印はリセットされない）
  - traffic_relation_id に紐づく信号機に対して全ての色検出結果がリセットされる
  - id は type:regulatory_element, subtype:traffic_light の relation id を指定すること
- use_traffic_light=true のとき (**not implemented**)
  - シミュレータ内での信号機オブジェクトの色をリセットする(未実装)

#### bool resetTrafficLightArrow(const int traffic_id, const bool use_traffic_light)

- use_traffic_light=false のとき
  - 指定された traffic_relation_idの矢印をリセットする（色はリセットされない）
  - traffic_relation_id に紐づく信号機に対して全ての矢印検出結果がリセットされる
  - id は type:regulatory_element, subtype:traffic_light の relation id を指定すること
- use_traffic_light=true のとき (**not implemented**)
  - シミュレータ内での信号機オブジェクトの矢印をリセットする(未実装)


#### bool getTrafficLightColor(const int traffic_id, std::string * traffic_color, const bool use_traffic_light)

- use_traffic_light=false のとき
  - 指定された traffic_relation_idに対して対応する現在の検出結果色を取得する("Red", "Yellow", "Green")
  - id は type:regulatory_element, subtype:traffic_light の relation id を指定すること
  - 対応する現在の検出結果色が取得でいない場合はfalseを返す
- use_traffic_light=true のとき (**not implemented**)
  - シミュレータ内での信号機オブジェクトの色を取得する(未実装)

#### bool getTrafficLightArrow(const int traffic_id, std::vector<std::string> * const traffic_Arrow, const bool use_traffic_light)

- use_traffic_light=false のとき
  - 指定された traffic_relation_idに対して対応する現在の検出結果矢印を取得する("Left", "Right", "Up")
  - 矢印の各方向に対応してstringのベクトルが返される
  - id は type:regulatory_element, subtype:traffic_light の relation id を指定すること
  - 対応する現在の検出結果矢印が取得でいない場合はfalseを返す
- use_traffic_light=true のとき (**not implemented**)
  - シミュレータ内での信号機オブジェクトの矢印を取得する(未実装)

#### bool getTrafficLineCenterPose(const int traffic_relation_id, geometry_msgs::Pose & line_pose)

- 指定された traffic_relation_idに紐づく白線（ストップライン）の中心のposeを返す
- poseのorientationには直近レーンのorientation（白線の垂線にあたる直線のorientation）が代入されている

#### bool getDistanceToTrafficLight(const int traffic_relation_id, double & distance)

- 現在位置からtraffic_relation_id に紐づく信号機のうち最も近いものとの距離を返す

#### bool getDistanceToTrafficLine(const int traffic_relation_id, double & distance)

- 現在位置からtraffic_relation_id に紐づく白線（ストップライン）との距離を返す

#### bool checkOverTrafficLine(const int traffic_relation_id, bool & over_line)

- 車両の先頭（base_linkではない）がtraffic_relation_id に紐づく白線（ストップライン）を超えているかどうかを返す
- 自車位置がストップラインから30m以内に入っていない場合は条件判定されず,白線を踏み越していない判定がなされる
  - ストップラインを30m以上超えた場合も同様である
- 値はover_lineに代入され、関数自体の返り値は正常判定なので注意

### util API

#### bool isObjectInArea(const std::string &name, geometry_msgs::Pose pose, double dist_thresh, double delta_yaw_thresh, const std::string & frame_type = "Center")

- 指定されたオブジェクトが指定された Pose に対して一定距離(dist_thresh)、一定角度(delta_yaw_thresh)以内にいるかどうかを判定する
- frame_typeとしては以下のものが指定可能である
  - "Cener" (default) ... 指定した座標とEgoCarのbase_link/NPCのポリゴン中心を比較する
  - "Front"          ... 指定した座標とEgoCar/NPCのポリゴン後端を比較する
  - "Rear"           ... 指定した座標とEgoCar/NPCのポリゴン前端を比較する

#### geometry_msgs::Pose genPoseROS(const double x, const double y, const double z, const double yaw)

- geometry_msgs::Pose を生成する

#### geometry_msgs::Pose genPoseROS(const double p_x, const double p_y, const double p_z, const double o_x, const double o_y, const double o_z, const double o_w)

- geometry_msgs::Poseを生成する




## sendGoalPointを送った際のNPCの仕様について
- 現在地点から送られたゴール地点までの最短距離を計算し、ルートを走行する
- sendCheckPointは受け付けない[TODO]sendCheckPoint API作成
- ゴールに到着した場合またはルートを逸脱した場合はその場で緩やかに停止する
- ルート走行に必要である場合でも、自動でレーンチェンジは行わない　[TODO]自動でレーンチェンジを行うオプションを追加する
  - そのため、レーンチェンジが必要な場面でレーンチェンジ指令を行わないと、レーンを逸脱し停止する挙動となる
- レーンチェンジや速度変更の指示は通常のNPCと同様受け付ける
  - ただし、ルート外のレーンIDに対するレーンチェンジ走行を指令した場合、レーンを逸脱し停止する挙動となる
- 右左折の指示は受け付けない（無視される）
