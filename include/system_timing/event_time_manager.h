//
// Created by irispete on 4/27/19.
//

#ifndef PROJECT_EVENTTIMEMANAGER_H
#define PROJECT_EVENTTIMEMANAGER_H

#include <iris_common/EventTime.h>
#include <iris_common/log/abstract_logger.h>

namespace system_timing
{
    class EventTimeManager
    {
        public:
            EventTimeManager(AbstractEventLogger& logger);
            EventTimeManager(std::function<void(std::list<iris_common::EventTime> &)> &save_sequence, AbstractEventLogger& logger);
            void addEvent(const iris_common::EventTime &event_time);
            void flush();
            void writeSequenceToLog(std::list<iris_common::EventTime>& sequence);
            void setSaveFunction(const std::function<void(std::list<iris_common::EventTime> &)> &save_sequence);

        private:
            std::list<iris_common::EventTime>& findSequence_(const iris_common::EventTime &event);
            std::function<void(std::list<iris_common::EventTime> &)> save_sequence_;
            std::map<std::string, std::map<int, std::list<iris_common::EventTime>>> sequences_;
            AbstractEventLogger& logger_;
    };
}


#endif //PROJECT_EVENTTIMEMANAGER_H
