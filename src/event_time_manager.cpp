//
// Created by irispete on 4/27/19.
//

#include "../include/system_timing/event_time_publisher.h"
#include "../include/system_timing/event_time_manager.h"

namespace system_timing
{

    EventTimeManager::EventTimeManager(AbstractEventLogger& logger)
        : logger_(logger)
    {
        if (!save_sequence_)
        {
            save_sequence_ =  [this](std::list<iris_common::EventTime>& event) {
                this->writeSequenceToLog(event);
            };
        }
    }

    EventTimeManager::EventTimeManager(std::function<void(std::list<iris_common::EventTime> &)> &save_sequence, AbstractEventLogger& logger)
        : EventTimeManager(logger)
    {
        save_sequence_ = save_sequence;
    }

    void EventTimeManager::addEvent(const iris_common::EventTime &event)
    {
        std::list<iris_common::EventTime> &sequence = findSequence_(event);
        switch (static_cast<EventTimeType>(event.type))
        {
            case EventTimeType::start:
            {
                if (sequence.size() > 0)
                {
                    logger_.error("restarting an existing timing sequence: deleting old events");
                    sequence.clear();
                }
                sequence.push_back(event);
            } break;
            case EventTimeType::add:
            {
                if (sequence.size() == 0)
                {
                    logger_.error("adding to a non-existing timing sequence");
                }
                else
                {
                    if (event.time < sequence.back().time)
                    {
                        logger_.error("event arrived out of order");
                    }
                }
                sequence.push_back(event);
            } break;
            case EventTimeType::end:
            {
                if (sequence.size() == 0)
                {
                    logger_.error("ending a non-existing timing sequence");
                }
                else
                {
                    if (event.time < sequence.back().time)
                    {
                        logger_.error("event arrived out of order");
                    }
                }
                sequence.push_back(event);
                save_sequence_(sequence);
                sequences_[event.category.data].erase(event.id);
            } break;
        }
    }

    void EventTimeManager::flush()
    {
        for (auto pair : sequences_)
        {
            for (auto id_pair : pair.second)
            {
                save_sequence_(id_pair.second);
            }
        }
    }

    void EventTimeManager::setSaveFunction(const std::function<void(std::list<iris_common::EventTime> &)> &save_sequence)
    {
        save_sequence_ = save_sequence;
    }

    void EventTimeManager::writeSequenceToLog(std::list<iris_common::EventTime>& sequence)
    {

    }

    std::list<iris_common::EventTime>& EventTimeManager::findSequence_(const iris_common::EventTime& event)
    {
         auto sequences_it = sequences_.find(event.category.data);
         if (sequences_it == sequences_.end())
         {
             auto category_sequences = std::map<int, std::list<iris_common::EventTime>>();
             sequences_[event.category.data] = category_sequences;
             auto sequence = std::list<iris_common::EventTime>();
             sequences_[event.category.data][event.id] = sequence;
             return sequences_[event.category.data][event.id];
         }
         return sequences_it->second[event.id];
    }

}

