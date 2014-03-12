/*
 * Copyright (C) 2013 Google Inc.
 * 
 * Licensed under the Apache License, Version 2.0 (the "License"); you may not
 * use this file except in compliance with the License. You may obtain a copy of
 * the License at
 * 
 * http://www.apache.org/licenses/LICENSE-2.0
 * 
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the
 * License for the specific language governing permissions and limitations under
 * the License.
 */

package org.ros.concurrent;

import org.ros.message.Time;

/**
 * A time-ordered buffer of objects. Ordering is defined by the result of a
 * supplied {@link TimeProvider}.
 * 
 * @author damonkohler@google.com (Damon Kohler)
 */
public class TimeOrderedBuffer<T> {

  private final TimeProvider<T> timeProvider;
  private final CircularBlockingDeque<T> deque;
  private final Object mutex;

  public interface TimeProvider<T> {
    /**
     * Returns the {@link Time} associated with the supplied {@link Object}.
     * 
     * @param object
     *          the {@link Object} to inspect
     * @return the {@link Time} associated with the supplied {@link Object}
     */
    Time get(T object);
  }

  public TimeOrderedBuffer(int capacity, TimeProvider<T> timeProvider) {
    this.timeProvider = timeProvider;
    deque = new CircularBlockingDeque<T>(capacity);
    mutex = new Object();
  }

  /**
   * Adds another object to the buffer. If the buffer has reached its capacity,
   * the oldest object is removed.
   * 
   * @param object
   *          the object to add to the buffer
   */
  public void add(T object) {
    deque.addFirst(object);
  }

  /**
   * Returns the object closest in time to the supplied time.
   * 
   * @param time
   *          the time to search for
   * @return the buffered object closest in time to {@code time} or {@code null}
   *         if the buffer is empty
   */
  public T get(Time time) {
    T result = null;
    synchronized (mutex) {
      long offset = 0;
      for (T object : deque) {
        if (result == null) {
          result = object;
          offset = Math.abs(time.subtract(timeProvider.get(result)).totalNsecs());
          continue;
        }
        long newOffset = Math.abs(time.subtract(timeProvider.get(object)).totalNsecs());
        if (newOffset < offset) {
          result = object;
          offset = newOffset;
        }
      }
    }
    if (result == null) {
      return null;
    }
    return result;
  }

  public T getLatest() {
    return deque.peekFirst();
  }
}
