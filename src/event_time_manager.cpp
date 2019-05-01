//
// Created by irispete on 4/27/19.
//

#include "../include/system_timing/event_time_publisher.h"
#include "../include/system_timing/event_time_manager.h"
#include <iris_common/log/csv_logger_inc.h>

namespace system_timing
{

    EventTimeManager::EventTimeManager(AbstractEventLogger& logger)
        : logger_(logger)
    {}

    EventTimeManager::~EventTimeManager()
    {
        flush();
    }

    void EventTimeManager::initialize(const std::string& output_folder)
    {
        event_time_csv_logger_.initialize(output_folder + "/events/");
        if (!save_sequence_)
        {
            save_sequence_ =  [this](std::list<iris_common::EventTime>& event) {
                this->writeSequenceToLog(event);
            };
        }
    }

    void EventTimeManager::addEvent(const iris_common::EventTime &event)
    {
        std::list<iris_common::EventTime> &sequence = findSequence_(event);
        switch (static_cast<EventTimeType>(event.type))
        {
            case EventTimeType::start:
            {
                addStartNode_(event, sequence);
            } break;
            case EventTimeType::add:
            {
                addNode_(event, sequence);
            } break;
            case EventTimeType::end:
            {
                addEndNode_(event, sequence);
            } break;
        }
        checkAndFlushOldEvents_(event);
    }

    void EventTimeManager::addStartNode_(const iris_common::EventTime &event,
                                         std::list<iris_common::EventTime> &sequence)
    {
        if (sequence.size() > 0)
        {
            logger_.error("restarting an existing timing sequence: deleting old events");
            sequence.clear();
        }
        sequence.push_back(std::move(event));
    }

    void EventTimeManager::addNode_(const iris_common::EventTime &event, std::list<iris_common::EventTime> &sequence)
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
    }

    void EventTimeManager::addEndNode_(const iris_common::EventTime &event, std::list<iris_common::EventTime> &sequence)
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
        if (save_sequence_)
        {
            save_sequence_(sequence);
        }
        sequences_[event.category.data].erase(event.id);
    }

    void EventTimeManager::flush()
    {
        if (save_sequence_)
        {
            for (auto pair : sequences_)
            {
                for (auto id_pair : pair.second)
                {
                    save_sequence_(id_pair.second);
                }
            }
        }
        sequences_.clear();
    }

    void EventTimeManager::checkAndFlushOldEvents_(const iris_common::EventTime &event)
    {
        // find the map of sequences for the category
        auto sequences_it = sequences_.find(event.category.data);
        if (sequences_it != sequences_.end())
        {
            auto& cat_sequences = sequences_it->second;
            // check for max sequences in category
            if (cat_sequences.size() > static_cast<unsigned long>(MAX_SEQUENCES))
            {
                // find the least (oldest) sequence in the category
                auto sequence_it = cat_sequences.begin();
                if (sequence_it != cat_sequences.end())
                {
                    auto& sequence = sequence_it->second;
                    // save and get rid of the oldest sequence
                    save_sequence_(sequence);
                    auto value = sequence_it->first;
                    cat_sequences.erase(value);
                }

            }
        }
    }

    void EventTimeManager::setSaveFunction(const std::function<void(std::list<iris_common::EventTime> &)> &save_sequence)
    {
        save_sequence_ = save_sequence;
    }

    void EventTimeManager::writeSequenceToLog(std::list<iris_common::EventTime>& sequence)
    {
        event_time_csv_logger_.write(sequence);
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

