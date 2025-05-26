CREATE TABLE robots (
    id INT PRIMARY KEY AUTO_INCREMENT,
    name VARCHAR(50) NOT NULL,
    namespace VARCHAR(64) NOT NULL,
    status ENUM('idle', 'working', 'error') NOT NULL DEFAULT 'idle',
    position_x FLOAT,
    position_y FLOAT,
    orientation FLOAT,
    battery INT,
    current_task_id INT,
    updated_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP ON UPDATE CURRENT_TIMESTAMP
);

-- 2. 차량 테이블 (로봇보다 먼저 필요함)
CREATE TABLE vehicles (
    id INT PRIMARY KEY AUTO_INCREMENT,
    plate_number VARCHAR(20) NOT NULL UNIQUE,
    company_name VARCHAR(100),
    status ENUM('waiting', 'loading', 'done') NOT NULL DEFAULT 'waiting',
    operation_type ENUM('inbound', 'outbound', 'both') NOT NULL DEFAULT 'both',
    arrival_time TIMESTAMP,
    departure_time TIMESTAMP
);

-- 3. 팔레트 테이블 (CHECK 제거됨)
CREATE TABLE pallets (
    id INT PRIMARY KEY AUTO_INCREMENT,
    barcode VARCHAR(64),
    slot ENUM('P1', 'P2', 'P3', 'P4', 'P5', 'P6'),
    layer ENUM('lower', 'upper'),
    status ENUM('stored', 'in_transit', 'loaded') NOT NULL DEFAULT 'stored',
    location ENUM('warehouse', 'robot', 'vehicle', 'unknown') NOT NULL DEFAULT 'unknown',
    robot_id INT,
    vehicle_id INT,
    updated_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP ON UPDATE CURRENT_TIMESTAMP,
    FOREIGN KEY (robot_id) REFERENCES robots(id) ON DELETE SET NULL,
    FOREIGN KEY (vehicle_id) REFERENCES vehicles(id) ON DELETE SET NULL
);

-- 4. 차량-팔레트 적재 테이블
CREATE TABLE vehicle_cargo (
    id INT PRIMARY KEY AUTO_INCREMENT,
    vehicle_id INT NOT NULL,
    pallet_id INT NOT NULL,
    loaded BOOLEAN DEFAULT FALSE,
    created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
    FOREIGN KEY (vehicle_id) REFERENCES vehicles(id) ON DELETE CASCADE,
    FOREIGN KEY (pallet_id) REFERENCES pallets(id) ON DELETE CASCADE
);

-- 5. 작업 테이블
CREATE TABLE tasks (
    id INT PRIMARY KEY AUTO_INCREMENT,
    robot_id INT,
    pallet_id INT NOT NULL,
    task_type ENUM('pickup', 'dropoff') NOT NULL,
    source_location VARCHAR(50) NOT NULL,
    destination_location VARCHAR(50) NOT NULL,
    status ENUM('pending', 'in_progress', 'completed', 'failed') NOT NULL DEFAULT 'pending',
    start_time TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
    end_time TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
    FOREIGN KEY (robot_id) REFERENCES robots(id) ON DELETE SET NULL,
    FOREIGN KEY (pallet_id) REFERENCES pallets(id) ON DELETE CASCADE
);

-- 6. 작업 중 사진 테이블
CREATE TABLE task_photos (
    id INT PRIMARY KEY AUTO_INCREMENT,
    task_id INT NOT NULL,
    robot_id INT NOT NULL,
    photo_type ENUM('pickup', 'dropoff') NOT NULL,
    photo_path VARCHAR(255) NOT NULL,
    description VARCHAR(128),
    timestamp TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
    FOREIGN KEY (task_id) REFERENCES tasks(id) ON DELETE CASCADE,
    FOREIGN KEY (robot_id) REFERENCES robots(id) ON DELETE CASCADE
);

-- 7. 로봇 상태 변경 로그
CREATE TABLE status_log (
    id INT PRIMARY KEY AUTO_INCREMENT,
    robot_id INT NOT NULL,
    status ENUM('idle', 'working', 'error') NOT NULL,
    timestamp TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
    FOREIGN KEY (robot_id) REFERENCES robots(id) ON DELETE CASCADE
);

-- 8. AI 기반 위치 추적 로그
CREATE TABLE localization_log (
    id INT PRIMARY KEY AUTO_INCREMENT,
    robot_id INT NOT NULL,
    pos_x FLOAT,
    pos_y FLOAT,
    orientation FLOAT,
    timestamp TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
    FOREIGN KEY (robot_id) REFERENCES robots(id) ON DELETE CASCADE
);

CREATE TABLE reservations (
    id INT PRIMARY KEY AUTO_INCREMENT,
    barcode VARCHAR(64) NOT NULL,
    quantity INT NOT NULL,
    date DATE NOT NULL,
    time TIME NOT NULL,
    operation_type ENUM('inbound', 'outbound') NOT NULL,
    vehicle_id INT,
    status ENUM('reserved', 'cancelled', 'completed') DEFAULT 'reserved',
    created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
    FOREIGN KEY (vehicle_id) REFERENCES vehicles(id) ON DELETE SET NULL
);

CREATE TABLE camera_events (
    id INT PRIMARY KEY AUTO_INCREMENT,
    event_type ENUM('plate_detected') NOT NULL,
    dock INT NOT NULL,                      -- 도크 번호 (1, 2 등)
    plate_number VARCHAR(20) NOT NULL,      -- 인식된 차량 번호
    confidence FLOAT,                       -- 인식 신뢰도
    timestamp TIMESTAMP DEFAULT CURRENT_TIMESTAMP
);




DELIMITER //
CREATE TRIGGER trg_check_pallet_ownership
BEFORE INSERT ON pallets
FOR EACH ROW
BEGIN
  IF NEW.robot_id IS NOT NULL AND NEW.vehicle_id IS NOT NULL THEN
    SIGNAL SQLSTATE '45000'
      SET MESSAGE_TEXT = '팔레트는 로봇 또는 차량 중 하나에만 연결될 수 있습니다.';
  END IF;
END;
//
DELIMITER ;

DELIMITER //
CREATE TRIGGER trg_check_pallet_ownership_update
BEFORE UPDATE ON pallets
FOR EACH ROW
BEGIN
  IF NEW.robot_id IS NOT NULL AND NEW.vehicle_id IS NOT NULL THEN
    SIGNAL SQLSTATE '45000'
      SET MESSAGE_TEXT = '팔레트는 로봇 또는 차량 중 하나에만 연결될 수 있습니다.';
  END IF;
END;
//
DELIMITER ;