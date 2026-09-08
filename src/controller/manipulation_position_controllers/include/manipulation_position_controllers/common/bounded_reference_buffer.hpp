// Copyright 2026
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include <optional>

#include <realtime_tools/realtime_thread_safe_box.hpp>

namespace manipulation_position_controllers::common
{

template<typename T>
class BoundedReferenceBuffer
{
public:
	void set(const T & value)
	{
		box_.set(value);
	}

	std::optional<T> try_get() const
	{
		return box_.try_get();
	}

	void clear()
	{
		box_.set(T{});
	}

private:
	mutable realtime_tools::RealtimeThreadSafeBox<T> box_;
};

}  // namespace manipulation_position_controllers::common