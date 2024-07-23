#include <apps_actuator/apps_actuator.hpp>

AppsActuator::AppsActuator() : EDFNode("apps_actuator_node")
{
	this->loadParameters();
    this->configureEDFScheduler(this->m_nPeriod, this->m_nWCET, this->m_nDeadline);
    this->openSpiDevice();

    this->m_subAppsActuator = this->create_subscription<std_msgs::msg::Int8>(
        this->m_sTopic, 1, std::bind(&AppsActuator::appsActuatorPercentageCallback, this, std::placeholders::_1));
    
    RCLCPP_INFO(
        this->get_logger(), "[ SPI INTERFACE ]: %s, [ TOPIC ]: %s, [ SPEED ]: %d", 
        this->m_sSpiInterface.c_str(),
        this->m_sTopic.c_str(),
        this->m_nSpiSpeed
    );
}

void AppsActuator::loadParameters() 
{
	declare_parameter("generic.spi_device", "");
	declare_parameter("topic.appsTopic", "");

	declare_parameter("generic.speed", 5000);
	declare_parameter("generic.WCET", 5000000);
	declare_parameter("generic.period", 10000000);
	declare_parameter("generic.deadline", 10000000);
	
	get_parameter("generic.spi_device", this->m_sSpiInterface);
	get_parameter("topic.appsTopic", this->m_sTopic);

	get_parameter("generic.speed", this->m_nSpiSpeed);
	get_parameter("generic.WCET", this->m_nWCET);
	get_parameter("generic.period", this->m_nPeriod);
	get_parameter("generic.deadline", this->m_nDeadline);
}

void AppsActuator::openSpiDevice()
{
    this->m_nIODevice = open(this->m_sSpiInterface.c_str(), O_RDWR);
    if (this->m_nIODevice < 0)
        RCLCPP_ERROR(this->get_logger(), "[ COULD NOT ENABLE DEVICE ]: %s", this->m_sSpiInterface.c_str());
    
    if (ioctl(this->m_nIODevice, SPI_IOC_WR_MODE, &this->m_nMode) < 0 || ioctl(this->m_nIODevice, SPI_IOC_RD_MODE, &this->m_nMode) < 0) 
    {
        RCLCPP_ERROR(this->get_logger(), "[ FAILED TO SET SPI MODE ]: %s", this->m_sSpiInterface.c_str());
        close(this->m_nIODevice);
    }

    if (ioctl(this->m_nIODevice, SPI_IOC_WR_BITS_PER_WORD, &this->m_nBits) < 0 || ioctl(this->m_nIODevice, SPI_IOC_RD_BITS_PER_WORD, &this->m_nBits) < 0) {
        RCLCPP_ERROR(this->get_logger(), "[ FAILED TO SET BITS PER WORD ]: %s", this->m_sSpiInterface.c_str());
        close(this->m_nIODevice);
    }

    if (ioctl(this->m_nIODevice, SPI_IOC_WR_MAX_SPEED_HZ, &this->m_nSpiSpeed) < 0 || ioctl(this->m_nIODevice, SPI_IOC_RD_MAX_SPEED_HZ, &this->m_nSpiSpeed) < 0) {
        RCLCPP_ERROR(this->get_logger(), "[ FAILED TO SET MAX SPEED ]: %s", this->m_sSpiInterface.c_str());
        close(this->m_nIODevice);
    }
}

void AppsActuator::appsActuatorPercentageCallback(const std_msgs::msg::Int8::SharedPtr appsTarget)
{
    RCLCPP_INFO(this->get_logger(), "[ TARGET APPS ]: %d", appsTarget->data);

    uint16_t avg =  (0x3FFF-0x3000)*appsTarget->data/100;
    avg += 0x3000;

    uint8_t msb = (uint8_t)(avg >> 8),  lsb = (uint8_t)avg;

    char message[16];
    strcpy(message, (const char*)&msb);
    strcat(message, (const char*)&lsb);

    size_t message_len = 16;
    uint8_t tx[message_len];
    uint8_t rx[message_len];

    memset(tx, 0, sizeof(tx));
    memset(rx, 0, sizeof(rx));
    strncpy((char *)tx, message, sizeof(tx) - 1);

    struct spi_ioc_transfer tr = {
        .tx_buf = (unsigned long)tx,
        .rx_buf = (unsigned long)rx,
        .len = message_len,
        .speed_hz = this->m_nSpiSpeed,
        .delay_usecs = this->m_nDelay,
        .bits_per_word = (unsigned char)this->m_nBits,
    };

    if (ioctl(this->m_nIODevice, SPI_IOC_MESSAGE(1), &tr) < 0)
        RCLCPP_ERROR(this->get_logger(), "[ FAILED TO SEND VALUE ON ]: %s, [ VALUE ]", this->m_sSpiInterface.c_str(), appsTarget->data);
}

void AppsActuator::timerCallback()
{
    auto time = std::chrono::steady_clock::now();
    // RCLCPP_INFO(this->get_logger(), "[ TIMESTAMP ]: %ld", std::chrono::duration_cast<std::chrono::seconds>(time.time_since_epoch()).count());
}

AppsActuator::~AppsActuator()
{
    close(this->m_nIODevice);
    RCLCPP_INFO(this->get_logger(), "CLOSING!");
}
