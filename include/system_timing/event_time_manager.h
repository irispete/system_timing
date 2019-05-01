//
// Created by irispete on 4/27/19.
//

#ifndef PROJECT_EVENTTIMEMANAGER_H
#define PROJECT_EVENTTIMEMANAGER_H

#include <system_timing/event_time_manager.h>
#include <system_timing/event_time_csv_logger.h>
#include <iris_common/EventTime.h>
#include <iris_common/log/abstract_logger.h>

namespace system_timing
{
    class EventTimeManager
    {
        public:
            EventTimeManager(AbstractEventLogger& logger);
            ~EventTimeManager();
            void initialize(const std::string& output_folder);
            void addEvent(const iris_common::EventTime &event_time);
            void flush();
            void writeSequenceToLog(std::list<iris_common::EventTime>& sequence);
            void setSaveFunction(const std::function<void(std::list<iris_common::EventTime> &)> &save_sequence);

        private:
            void addStartNode_(const iris_common::EventTime &event, std::list<iris_common::EventTime> &sequence);
            void addNode_(const iris_common::EventTime &event, std::list<iris_common::EventTime> &sequence);
            void addEndNode_(const iris_common::EventTime &event, std::list<iris_common::EventTime> &sequence);
            void checkAndFlushOldEvents_(const iris_common::EventTime &event);
            std::list<iris_common::EventTime>& findSequence_(const iris_common::EventTime &event);

            const int MAX_SEQUENCES = 60; // 4 seconds worth at 15 fps
            std::function<void(std::list<iris_common::EventTime> &)> save_sequence_;
            std::map<std::string, std::map<int, std::list<iris_common::EventTime>>> sequences_;
            AbstractEventLogger& logger_;
            EventTimeCsvLogger event_time_csv_logger_;

    };
}


#endif //PROJECT_EVENTTIMEMANAGER_H
