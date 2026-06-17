/*
 * Robot.h
 *
 *  Created on: Nov 23, 2025
 *      Author: Dell
 */
#include"main.h"
#ifndef SRC_ROBOT_H_
#define SRC_ROBOT_H_

class Robot {
private:
	int speed;
public:
	Robot(int speed);
	virtual ~Robot();
	Robot(const Robot &other);
	Robot(Robot &&other);
};

#endif /* SRC_ROBOT_H_ */
