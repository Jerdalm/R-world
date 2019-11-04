#include "Robot.hpp"
#include <sstream>
#include <ctime>
#include <chrono>
#include "Thread.hpp"
#include "MathUtils.hpp"
#include "Logger.hpp"
#include "Wall.hpp"
#include "Goal.hpp"
#include "WayPoint.hpp"
#include "RobotWorld.hpp"
#include "Shape2DUtils.hpp"
#include "CommunicationService.hpp"
#include "Client.hpp"
#include "Message.hpp"
#include "MainApplication.hpp"
#include "LaserDistanceSensor.hpp"
#include <thread>
#include <mutex>
#include <chrono>
//#include <windows.h>
#include "Client.hpp"
#include "Message.hpp"
#include <stdio.h>
#include <math.h>

namespace Model
{
	/**
	 *
	 */
	Robot::Robot() :
								name( ""),
								size( DefaultSize),
								position( DefaultPosition),
								front( 0, 0),
								speed( 0.0),
								acting(false),
								driving(false),
								communicating(false)
	{
		std::shared_ptr< AbstractSensor > laserSensor( new LaserDistanceSensor( this));
		attachSensor( laserSensor);
	}
	/**
	 *
	 */
	Robot::Robot( const std::string& aName) :
								name( aName),
								size( DefaultSize),
								position( DefaultPosition),
								front( 0, 0),
								speed( 0.0),
								acting(false),
								driving(false),
								communicating(false)
	{
		std::shared_ptr< AbstractSensor > laserSensor( new LaserDistanceSensor( this));
		attachSensor( laserSensor);
	}
	/**
	 *
	 */
	Robot::Robot(	const std::string& aName,
					const Point& aPosition) :
								name( aName),
								size( DefaultSize),
								position( aPosition),
								front( 0, 0),
								speed( 0.0),
								acting(false),
								driving(false),
								communicating(false)
	{
		std::shared_ptr< AbstractSensor > laserSensor( new LaserDistanceSensor( this));
		attachSensor( laserSensor);
	}
	/**
	 *
	 */
	Robot::~Robot()
	{
		if(driving)
		{
			stopDriving();
		}
		if(acting)
		{
			stopActing();
		}
		if(communicating)
		{
			stopCommunicating();
		}
	}
	/**
	 *
	 */
	void Robot::setName( const std::string& aName,
						 bool aNotifyObservers /*= true*/)
	{
		name = aName;
		if (aNotifyObservers == true)
		{
			notifyObservers();
		}

	}
	/**
	 *
	 */
	Size Robot::getSize() const
	{
		return size;
	}
	/**
	 *
	 */
	void Robot::setSize(	const Size& aSize,
							bool aNotifyObservers /*= true*/)
	{
		size = aSize;
		if (aNotifyObservers == true)
		{
			notifyObservers();
		}
	}
	/**
	 *
	 */
	void Robot::setPosition(	const Point& aPosition,
								bool aNotifyObservers /*= true*/)
	{
		position = aPosition;
		if (aNotifyObservers == true)
		{
			notifyObservers();
		}
	}
	/**
	 *
	 */
	BoundedVector Robot::getFront() const
	{
		return front;
	}
	/**
	 *
	 */
	void Robot::setFront(	const BoundedVector& aVector,
							bool aNotifyObservers /*= true*/)
	{
		front = aVector;
		if (aNotifyObservers == true)
		{
			notifyObservers();
		}
	}
	/**
	 *
	 */
	float Robot::getSpeed() const
	{
		return speed;
	}
	/**
	 *
	 */
	void Robot::setSpeed( float aNewSpeed,
						  bool aNotifyObservers /*= true*/)
	{
		speed = aNewSpeed;
		if (aNotifyObservers == true)
		{
			notifyObservers();
		}
	}
	/**
	 *
	 */
	void Robot::startActing()
	{
		acting = true;
		std::thread newRobotThread( [this]{	startDriving();});
		robotThread.swap( newRobotThread);
	}
	/**
	 *
	 */
	void Robot::stopActing()
	{
		acting = false;
		driving = false;
		robotThread.join();
	}
	/**
	 *
	 */
	void Robot::startDriving()
	{
		driving = true;

		goal = RobotWorld::getRobotWorld().getGoal( "Goal");
		calculateRoute(goal);
		stuurBericht(StartDriving, "start");
		while (waiting)
		{
			//Sleep(10);
		}
		drive();
	}
	/**
	 *
	 */
	void Robot::stopDriving()
	{
		driving = false;
	}
	/**
	 *
	 */
	void Robot::startCommunicating()
	{
		if(!communicating)
		{
			communicating = true;


			std::string localPort = "12345";
			if (Application::MainApplication::isArgGiven( "-local_port"))
			{
				localPort = Application::MainApplication::getArg( "-local_port").value;
			}

			Messaging::CommunicationService::getCommunicationService().runRequestHandler( toPtr<Robot>(),
																						  static_cast<unsigned short>(std::stoi(localPort)));

		}
	}
	/**
	 *
	 */
	void Robot::stopCommunicating()
	{
		if(communicating)
		{
			communicating = false;

			std::string localPort = "12345";
			if (Application::MainApplication::isArgGiven( "-local_port"))
			{
				localPort = Application::MainApplication::getArg( "-local_port").value;
			}

			Messaging::Client c1ient( 	"localhost",
										localPort,
										toPtr<Robot>());
			Messaging::Message message( 1, "stop");
			c1ient.dispatchMessage( message);
		}
	}
	/**
	 *
	 */
	Region Robot::getRegion() const
	{
		Point translatedPoints[] = { getFrontRight(), getFrontLeft(), getBackLeft(), getBackRight() };
		return Region( 4, translatedPoints);
	}
	/**
	 *
	 */
	bool Robot::intersects( const Region& aRegion) const
	{
		Region region = getRegion();
		region.Intersect( aRegion);
		return !region.IsEmpty();
	}
	/**
	 *
	 */
	Point Robot::getFrontLeft() const
	{
		// x and y are pointing to top left now
		int x = position.x - (size.x / 2);
		int y = position.y - (size.y / 2);

		Point originalFrontLeft( x, y);
		double angle = Utils::Shape2DUtils::getAngle( front) + 0.5 * Utils::PI;

		Point frontLeft( static_cast<int>((originalFrontLeft.x - position.x) * std::cos( angle) - (originalFrontLeft.y - position.y) * std::sin( angle) + position.x),
						 static_cast<int>((originalFrontLeft.y - position.y) * std::cos( angle) + (originalFrontLeft.x - position.x) * std::sin( angle) + position.y));

		return frontLeft;
	}
	/**
	 *
	 */
	Point Robot::getFrontRight() const
	{
		// x and y are pointing to top left now
		int x = position.x - (size.x / 2);
		int y = position.y - (size.y / 2);

		Point originalFrontRight( x + size.x, y);
		double angle = Utils::Shape2DUtils::getAngle( front) + 0.5 * Utils::PI;

		Point frontRight( static_cast<int>((originalFrontRight.x - position.x) * std::cos( angle) - (originalFrontRight.y - position.y) * std::sin( angle) + position.x),
						  static_cast<int>((originalFrontRight.y - position.y) * std::cos( angle) + (originalFrontRight.x - position.x) * std::sin( angle) + position.y));

		return frontRight;
	}
	/**
	 *
	 */
	Point Robot::getBackLeft() const
	{
		// x and y are pointing to top left now
		int x = position.x - (size.x / 2);
		int y = position.y - (size.y / 2);

		Point originalBackLeft( x, y + size.y);

		double angle = Utils::Shape2DUtils::getAngle( front) + 0.5 * Utils::PI;

		Point backLeft( static_cast<int>((originalBackLeft.x - position.x) * std::cos( angle) - (originalBackLeft.y - position.y) * std::sin( angle) + position.x),
						static_cast<int>((originalBackLeft.y - position.y) * std::cos( angle) + (originalBackLeft.x - position.x) * std::sin( angle) + position.y));

		return backLeft;

	}
	/**
	 *
	 */
	Point Robot::getBackRight() const
	{
		// x and y are pointing to top left now
		int x = position.x - (size.x / 2);
		int y = position.y - (size.y / 2);

		Point originalBackRight( x + size.x, y + size.y);

		double angle = Utils::Shape2DUtils::getAngle( front) + 0.5 * Utils::PI;

		Point backRight( static_cast<int>((originalBackRight.x - position.x) * std::cos( angle) - (originalBackRight.y - position.y) * std::sin( angle) + position.x),
						 static_cast<int>((originalBackRight.y - position.y) * std::cos( angle) + (originalBackRight.x - position.x) * std::sin( angle) + position.y));

		return backRight;
	}
	/**
	 *
	 */
	void Robot::handleNotification()
	{
		//	std::unique_lock<std::recursive_mutex> lock(robotMutex);

		static int update = 0;
		if ((++update % 200) == 0)
		{
			notifyObservers();
		}
	}
	/**
	 *
	 */
	void Robot::handleRequest( Messaging::Message& aMessage)
	{
		Application::Logger::log( __PRETTY_FUNCTION__ + aMessage.asString());
		switch(aMessage.getMessageType())
		{
			case EchoRequest:
			{
				//Application::Logger::log( __PRETTY_FUNCTION__ + std::string(": EchoRequest"));
				aMessage.setMessageType(EchoResponse);
				aMessage.setBody( ": case 1 " + aMessage.asString());
				break;
			}
			case CopyWorld:
			{
				Application::Logger::log( __PRETTY_FUNCTION__ + std::string(": CopyWorld"));
				Application::Logger::log(aMessage.asString());
				RobotWorld::getRobotWorld().copyWorld(aMessage.asString());
				wontWait = false;

				Model::RobotPtr robot = Model::RobotWorld::getRobotWorld().getRobot( "Robot");
				Model::GoalPtr goal = Model::RobotWorld::getRobotWorld().getGoal( "Goal");
				aMessage.setMessageType(CopyWorldResponse);
				aMessage.setBody(robot->asString() + goal->asString());
				break;
			}
			case StartRobot:
			{
				Model::RobotPtr robot = Model::RobotWorld::getRobotWorld().getRobot( "Robot");
				if (robot && !robot->isActing())
				{
					robot->startActing();
				}
				break;
			}
			case StartDriving:
			{
				waiting = false;
				//aMessage.setMessageType(StartDrivingResponse);
				break;
			}
			case UpdatePosition:
			{
				RobotWorld::getRobotWorld().moveRobot2(aMessage.asString());
				break;
			}
			default:
			{
				Application::Logger::log( __PRETTY_FUNCTION__ + std::string(": default"));

				aMessage.setBody( " default  Goodbye cruel world!");
				break;
			}
		}
	}
	/**
	 *
	 */
	void Robot::handleResponse( const Messaging::Message& aMessage)
	{
		switch(aMessage.getMessageType())
		{
			case EchoResponse:
			{
				Application::Logger::log( __PRETTY_FUNCTION__ + std::string( "") + aMessage.asString());

				break;
			}
			case CopyWorldResponse:
			{
				Application::Logger::log( __PRETTY_FUNCTION__ );
				RobotWorld::getRobotWorld().copyWorld(aMessage.asString());

				break;
			}
			default:
			{
				//Application::Logger::log( __PRETTY_FUNCTION__ + std::string( ": default not implemented, ") + aMessage.asString());
				break;
			}
		}
	}
	/**
	 *
	 */
	std::string Robot::asString() const
	{
		std::ostringstream os;

		os << "Robot " << name << " at (" << position.x << "," << position.y << ")";

		return os.str();
	}
	/**
	 *
	 */
	std::string Robot::asDebugString() const
	{
		std::ostringstream os;

		os << "Robot:\n";
		os << AbstractAgent::asDebugString();
		os << "Robot " << name << " at (" << position.x << "," << position.y << ")\n";

		return os.str();
	}
	/**
	 *
	 */
	void Robot::drive()
	{
		try
		{
			for (std::shared_ptr< AbstractSensor > sensor : sensors)
			{
				//sensor->setOn();
			}

			if (speed == 0.0)
			{
				speed = 10.0;
			}

			unsigned pathPoint = 0;
			while (position.x > 0 && position.x < 500 && position.y > 0 && position.y < 500 && pathPoint < path.size())
			{
				const PathAlgorithm::Vertex& vertex = path[pathPoint+=static_cast<int>(speed)];
				front = BoundedVector( vertex.asPoint(), position);
				position.x = vertex.x;
				position.y = vertex.y;
				std::time_t turnTime;

				if(collision())
				{
					turnTime = std::time(nullptr);
					Application::Logger::log(__PRETTY_FUNCTION__ + std::string(": collision"));
					//notifyObservers();
					speed = -10.0;

				}


				std::time_t restartTime = std::time(nullptr);

				if(restartTime - turnTime >= 1){
					speed = 10.0;
				}

				if (arrived(goal))
				{
					Application::Logger::log(__PRETTY_FUNCTION__ + std::string(": arrived or collision"));
					notifyObservers();
					break;
				}

				notifyObservers();

				std::this_thread::sleep_for( std::chrono::milliseconds( 100));

				// this should be the last thing in the loop
				if(driving == false)
				{
					return;
				}
				stuurBericht(UpdatePosition, asString());
			} // while

			for (std::shared_ptr< AbstractSensor > sensor : sensors)
			{
				//sensor->setOff();
			}
		}
		catch (std::exception& e)
		{
			std::cerr << __PRETTY_FUNCTION__ << ": " << e.what() << std::endl;
		}
		catch (...)
		{
			std::cerr << __PRETTY_FUNCTION__ << ": unknown exception" << std::endl;
		}
	}
	/**
	 *
	 */
	void Robot::calculateRoute(GoalPtr aGoal)
	{
		path.clear();
		if (aGoal)
		{
			// Turn off logging if not debugging AStar
			Application::Logger::setDisable();

			front = BoundedVector( aGoal->getPosition(), position);
			handleNotificationsFor( astar);
			path = astar.search( position, aGoal->getPosition(), size);
			stopHandlingNotificationsFor( astar);

			Application::Logger::setDisable( false);
			routeFound = true;
		}
	}
	/**
	 *
	 */
	bool Robot::arrived(GoalPtr aGoal)
	{
		if (aGoal && intersects( aGoal->getRegion()))
		{
			return true;
		}
		return false;
	}
	/**
	 *
	 */
	bool Robot::collision()
	{
		auto Deviation = 40;
		Point frontLeft = Point(getFrontLeft().x+Deviation, getFrontLeft().y+Deviation);
		Point frontRight = Point(getFrontRight().x+Deviation, getFrontRight().y-Deviation);
		Point backLeft = Point(getBackLeft().y-Deviation, getBackLeft().y+Deviation);
		Point backRight = Point(getBackRight().x-Deviation, getBackRight().y-Deviation);

		const std::vector< WallPtr >& walls = RobotWorld::getRobotWorld().getWalls();
		Model::RobotPtr robot2 = Model::RobotWorld::getRobotWorld().getRobot( "Robot2");
		for (WallPtr wall : walls)
		{
			if (Utils::Shape2DUtils::intersect( frontLeft, frontRight, wall->getPoint1(), wall->getPoint2()) ||
							Utils::Shape2DUtils::intersect( frontLeft, backLeft, wall->getPoint1(), wall->getPoint2())	||
							Utils::Shape2DUtils::intersect( frontRight, backRight, wall->getPoint1(), wall->getPoint2()))
			{
				return true;
			}
		}
		Point een = Point(position.x + size.y/2 + 20, position.y - size.y/2 - 20);
		Point twee = Point(position.x - size.y/2 - 20, position.y + size.y/2 + 20);
		Point drie = Point(robot2->getPosition().x - robot2->getSize().y/2 - 20, robot2->getPosition().y - robot2->getSize().y/2 - 20);
		Point vier = Point(robot2->getPosition().x + robot2->getSize().y/2 + 20, robot2->getPosition().y + robot2->getSize().y/2 + 20);
		Point vijf = Point(een.x-size.y, een.y);
		Point zes = Point(twee.x+size.y, twee.y);
		Point zeven = Point(drie.x+robot2->getSize().y, drie.y);
		Point acht = Point(vier.x-robot2->getSize().y, vier.y);
		if (Utils::Shape2DUtils::intersect( een, twee, drie, vier) || Utils::Shape2DUtils::intersect( vijf, zes, zeven, acht))
		{
			if (wontWait == false)
			{
				return true;
			}
		}
		return false;
	}

	void Robot::stuurBericht(Model::Robot::MessageType type, std::string data)
		{
			Model::RobotPtr robot = Model::RobotWorld::getRobotWorld().getRobot( "Robot");
			if (robot)
					{
						std::string remoteIpAdres = "localhost";
						std::string remotePort = "12345";

						if (Application::MainApplication::isArgGiven( "-remote_ip"))
						{
							remoteIpAdres = Application::MainApplication::getArg( "-remote_ip").value;
						}
						if (Application::MainApplication::isArgGiven( "-remote_port"))
						{
							remotePort = Application::MainApplication::getArg( "-remote_port").value;
						}
						// We will request an echo message. The response will be "Hello World", if all goes OK,
						// "Goodbye cruel world!" if something went wrong.
						Messaging::Client c1ient( remoteIpAdres,
												  remotePort,
												  robot);
						Messaging::Message message(type, data);
						c1ient.dispatchMessage( message);
					}
		}

} // namespace Model
