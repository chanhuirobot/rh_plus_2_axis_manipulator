
#ifndef TWOMANI_SERIAL__H
#define TWOMANI_SERIAL__H
#include <hidapi/hidapi.h>
#include <sstream>
#include <vector>

#include "twomani_drvr.hpp"

namespace twomani
{
	class twomani_serial: public twomani_drvr
	{
		public:
			twomani_serial();
			~twomani_serial();

			bool open(const std::string &portname) override;
			void close() override;

			bool getJointPosition(int id, uint16_t &pos) override;
			bool setJointPosition(int id, uint16_t pos, uint16_t time) override;
			bool setManualModeAll(bool enable, int count) override;

		private:
			int fd_;
	};
}

#endif // TWOMANI_SERIAL__H
