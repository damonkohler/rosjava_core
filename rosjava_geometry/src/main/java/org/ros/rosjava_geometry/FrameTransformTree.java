/*
 * Copyright (C) 2011 Google Inc.
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

package org.ros.rosjava_geometry;

import com.google.common.annotations.VisibleForTesting;
import com.google.common.base.Preconditions;
import com.google.common.collect.Maps;

import geometry_msgs.TransformStamped;
import org.ros.concurrent.TimeOrderedBuffer;
import org.ros.message.Time;
import org.ros.namespace.GraphName;

import java.util.Map;

/**
 * A tree of {@link FrameTransform}s.
 * <p>
 * {@link FrameTransformTree} does not currently support time travel. Lookups
 * always use the newest {@link TransformStamped}.
 * 
 * @author damonkohler@google.com (Damon Kohler)
 * @author moesenle@google.com (Lorenz Moesenlechner)
 */
public class FrameTransformTree {

  private static final int TRANSFORM_QUEUE_CAPACITY = 16;

  private final Object mutex;

  /**
   * Forces a relative GraphName to add support for tf2 while maintaining
   * backward compatibility with tf.
   */
  private class LazyFrameTransformBuffers {
    /**
     * A {@link Map} of the most recent {@link LazyFrameTransform} by source
     * frame. Lookups by target frame or by the pair of source and target are
     * both unnecessary because every frame can only have exactly one target.
     */
    private final Map<GraphName, TimeOrderedBuffer<LazyFrameTransform>> buffers;

    public LazyFrameTransformBuffers() {
      buffers = Maps.newConcurrentMap();
    }

    public TimeOrderedBuffer<LazyFrameTransform> get(GraphName source) {
      return buffers.get(source.toRelative());
    }

    public void add(GraphName source, LazyFrameTransform lazyFrameTransform) {
      GraphName relativeSource = source.toRelative();
      if (!buffers.containsKey(relativeSource)) {
        buffers.put(relativeSource, new TimeOrderedBuffer<LazyFrameTransform>(
            TRANSFORM_QUEUE_CAPACITY, new TimeOrderedBuffer.TimeProvider<LazyFrameTransform>() {
              @Override
              public Time get(LazyFrameTransform lazyFrameTransform) {
                return lazyFrameTransform.get().getTime();
              }
            }));
      }
      synchronized (mutex) {
        buffers.get(relativeSource).add(lazyFrameTransform);
      }
    }
  }

  LazyFrameTransformBuffers buffers;

  public FrameTransformTree() {
    mutex = new Object();
    buffers = new LazyFrameTransformBuffers();
  }

  /**
   * Updates the tree with the provided {@link geometry_msgs.TransformStamped}
   * message.
   * <p>
   * Note that the tree is updated lazily. Modifications to the provided
   * {@link geometry_msgs.TransformStamped} message may cause unpredictable
   * results.
   * 
   * @param transformStamped
   *          the {@link geometry_msgs.TransformStamped} message to update with
   */
  public void update(geometry_msgs.TransformStamped transformStamped) {
    Preconditions.checkNotNull(transformStamped);
    GraphName source = GraphName.of(transformStamped.getChildFrameId());
    LazyFrameTransform lazyFrameTransform = new LazyFrameTransform(transformStamped);
    buffers.add(source, lazyFrameTransform);
  }

  @VisibleForTesting
  void update(FrameTransform frameTransform) {
    Preconditions.checkNotNull(frameTransform);
    GraphName source = frameTransform.getSourceFrame();
    LazyFrameTransform lazyFrameTransform = new LazyFrameTransform(frameTransform);
    buffers.add(source, lazyFrameTransform);
  }

  /**
   * Returns the most recent {@link FrameTransform} for target {@code source}.
   * 
   * @param source
   *          the frame to look up
   * @return the most recent {@link FrameTransform} for {@code source} or
   *         {@code null} if no transform for {@code source} is available
   */
  public FrameTransform lookUp(GraphName source) {
    Preconditions.checkNotNull(source);
    return getLatest(source);
  }

  private FrameTransform getLatest(GraphName source) {
    TimeOrderedBuffer<LazyFrameTransform> timeOrderedBuffer = buffers.get(source);
    if (timeOrderedBuffer == null) {
      return null;
    }
    LazyFrameTransform lazyFrameTransform = timeOrderedBuffer.getLatest();
    if (lazyFrameTransform == null) {
      return null;
    }
    return lazyFrameTransform.get();
  }

  /**
   * @see #lookUp(GraphName)
   */
  public FrameTransform get(String source) {
    Preconditions.checkNotNull(source);
    return lookUp(GraphName.of(source));
  }

  /**
   * Returns the {@link FrameTransform} for {@code source} closest to
   * {@code time}.
   * 
   * @param source
   *          the frame to look up
   * @param time
   *          the transform for {@code frame} closest to this {@link Time} will
   *          be returned
   * @return the most recent {@link FrameTransform} for {@code source} or
   *         {@code null} if no transform for {@code source} is available
   */
  public FrameTransform lookUp(GraphName source, Time time) {
    Preconditions.checkNotNull(source);
    Preconditions.checkNotNull(time);
    TimeOrderedBuffer<LazyFrameTransform> timeOrderedBuffer = buffers.get(source);
    if (timeOrderedBuffer == null) {
      return null;
    }
    LazyFrameTransform lazyFrameTransform = timeOrderedBuffer.get(time);
    if (lazyFrameTransform == null) {
      return null;
    }
    return lazyFrameTransform.get();
  }

  /**
   * @see #lookUp(GraphName, Time)
   */
  public FrameTransform get(String source, Time time) {
    Preconditions.checkNotNull(source);
    return lookUp(GraphName.of(source), time);
  }

  /**
   * @param time TODO
   * @return the {@link FrameTransform} from source the frame to the target
   *         frame, or {@code null} if no {@link FrameTransform} could be found
   */
  public FrameTransform transform(GraphName source, GraphName target, Time time) {
    Preconditions.checkNotNull(source);
    Preconditions.checkNotNull(target);
    // This adds support for tf2 while maintaining backward compatibility with
    // tf.
    GraphName relativeSource = source.toRelative();
    GraphName relativeTarget = target.toRelative();
    if (relativeSource.equals(relativeTarget)) {
      return new FrameTransform(Transform.identity(), relativeSource, relativeTarget, time);
    }
    FrameTransform sourceToRoot = transformToRoot(relativeSource, time);
    FrameTransform targetToRoot = transformToRoot(relativeTarget, time);
    if (sourceToRoot == null && targetToRoot == null) {
      return null;
    }
    if (sourceToRoot == null) {
      if (targetToRoot.getTargetFrame().equals(relativeSource)) {
        // relativeSource is root.
        return targetToRoot.invert();
      } else {
        return null;
      }
    }
    if (targetToRoot == null) {
      if (sourceToRoot.getTargetFrame().equals(relativeTarget)) {
        // relativeTarget is root.
        return sourceToRoot;
      } else {
        return null;
      }
    }
    if (sourceToRoot.getTargetFrame().equals(targetToRoot.getTargetFrame())) {
      // Neither relativeSource nor relativeTarget is root and both share the
      // same root.
      Transform transform =
          targetToRoot.getTransform().invert().multiply(sourceToRoot.getTransform());
      return new FrameTransform(transform, relativeSource, relativeTarget, sourceToRoot.getTime());
    }
    // No known transform.
    return null;
  }

  /**
   * @param time TODO
   * @see #transform(GraphName, GraphName, Time)
   */
  public FrameTransform transform(String source, String target, Time time) {
    Preconditions.checkNotNull(source);
    Preconditions.checkNotNull(target);
    return transform(GraphName.of(source), GraphName.of(target), time);
  }

  /**
   * @param source
   *          the source frame
   * @param time TODO
   * @return the {@link Transform} from {@code source} to root
   */
  @VisibleForTesting
  FrameTransform transformToRoot(GraphName source, Time time) {
    Preconditions.checkArgument(source.isRelative());
    FrameTransform result = lookUp(source, time);
    if (result == null) {
      return null;
    }
    while (true) {
      FrameTransform resultToParent = lookUp(result.getTargetFrame(), result.getTime());
      if (resultToParent == null) {
        return result;
      }
      // Now resultToParent.getSourceFrame() == result.getTargetFrame()
      Transform transform = resultToParent.getTransform().multiply(result.getTransform());
      GraphName target = resultToParent.getTargetFrame();
      result = new FrameTransform(transform, source, target, result.getTime());
    }
  }
}
