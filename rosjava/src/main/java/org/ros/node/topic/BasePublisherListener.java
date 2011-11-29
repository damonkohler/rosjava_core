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

package org.ros.node.topic;

/**
 * A {@link PublisherListener} which provides empty defaults for all messages.
 *
 * @author Keith M. Hughes
 */
public class BasePublisherListener implements PublisherListener {

  @Override
  public void onPublisherMasterRegistration(Publisher<?> publisher) {
    // Default is do nothing.
  }

  @Override
  public void onPublisherRemoteConnection(Publisher<?> publisher) {
    // Default is do nothing.
  }

  @Override
  public void onPublisherShutdown(Publisher<?> publisher) {
	// Default is do nothing.
  }

}
