/*
 * Copyright (c) 2023, Uppsala universitet.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the Institute nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE INSTITUTE AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE INSTITUTE OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 *
 */

package org.contikios.cooja.radiomediums;

import java.util.Collection;
import java.util.HashMap;
import java.util.HashSet;
import java.util.Map;
import java.util.Random;
import java.util.Set;

import org.contikios.cooja.ClassDescription;
import org.contikios.cooja.Simulation;
import org.contikios.cooja.interfaces.Radio;
import org.jdom2.Element;

@ClassDescription("FrequencySelectiveUDGM")
public class FrequencySelectiveUDGM extends UDGM {
  private static final int MIN_CHANNEL = 11;
  private static final int MAX_CHANNEL = 26;
  private static final int CHANNELS_COUNT = MAX_CHANNEL - MIN_CHANNEL + 1;
  private double worstChannelsSuccessRatio = 0.7;
  private double bestChannelsSuccessRatio = 0.99;
  private Map<Radio, Map<Radio, double[]>> edgeToSuccessRatios;

  public FrequencySelectiveUDGM(Simulation simulation) {
    super(simulation);
    edgeToSuccessRatios = new HashMap<>();
  }

  @Override
  public double getTxSuccessProbability() {
    return 1.0d;
  }

  private double[] generateSuccessRatios() {
    Random random = simulation.getRandomGenerator();
    double sucessRatios[] = new double[CHANNELS_COUNT];
    Set<Integer> setIndices = new HashSet<>();
    double nextSucessRatio = worstChannelsSuccessRatio;
    double stepSize = (bestChannelsSuccessRatio - worstChannelsSuccessRatio)
        / (CHANNELS_COUNT - 1);

    while(setIndices.size() < CHANNELS_COUNT) {
      int index;
      do {
        index = random.nextInt(CHANNELS_COUNT);
      } while(setIndices.contains(index));
      setIndices.add(index);
      sucessRatios[index] = nextSucessRatio;
      nextSucessRatio += stepSize;
    }
    return sucessRatios;
  }

  private Map<Radio, double[]> getOrCreateSuccessRatiosMap(Radio radio) {
    Map<Radio, double[]> sucessRatiosMap = edgeToSuccessRatios.get(radio);
    if (sucessRatiosMap == null) {
      sucessRatiosMap = new HashMap<Radio, double[]>();
      edgeToSuccessRatios.put(radio, sucessRatiosMap);
    }
    return sucessRatiosMap;
  }

  private double[] getOrCreateSuccessRatiosBetween(Radio a, Radio b) {
    Map<Radio, double[]> asSuccessRatios = getOrCreateSuccessRatiosMap(a);
    Map<Radio, double[]> bsSuccessRatios = getOrCreateSuccessRatiosMap(b);

    double[] sucessRatios = asSuccessRatios.get(b);
    if(sucessRatios != null) {
      return sucessRatios;
    }

    sucessRatios = bsSuccessRatios.get(a);
    if(sucessRatios != null) {
      return sucessRatios;
    }

    sucessRatios = generateSuccessRatios();
    asSuccessRatios.put(b, sucessRatios);
    return sucessRatios;
  }

  @Override
  public double getRxSuccessProbability(Radio source, Radio dest) {
    if(super.getRxSuccessProbability(source, dest) == 0.0d) {
      /* not in range */
      return 0.0d;
    }

    double[] successRatios = getOrCreateSuccessRatiosBetween(source, dest);
    return successRatios[source.getChannel() - MIN_CHANNEL];
  }

  public void deteriorateLink(Radio a, Radio b) {
    double[] sucessRatios = getOrCreateSuccessRatiosBetween(a, b);
    for (int i = 0; i < sucessRatios.length; i++) {
      sucessRatios[i] = sucessRatios[i] * 0.9;
    }
  }

  public void improveLink(Radio a, Radio b) {
    double[] sucessRatios = getOrCreateSuccessRatiosBetween(a, b);
    for (int i = 0; i < sucessRatios.length; i++) {
      sucessRatios[i] = sucessRatios[i] / 0.9;
    }
  }

  @Override
  public Collection<Element> getConfigXML() {
    Collection<Element> config = super.getConfigXML();
    Element element;

    element = new Element("worst_channels_success_ratio");
    element.setText(Double.toString(worstChannelsSuccessRatio));
    config.add(element);

    element = new Element("best_channels_success_ratio");
    element.setText(Double.toString(bestChannelsSuccessRatio));
    config.add(element);

    return config;
  }

  @Override
  public boolean setConfigXML(Collection<Element> configXML, boolean visAvailable) {
    super.setConfigXML(configXML, visAvailable);
    for (Element element : configXML) {
      if (element.getName().equals("worst_channels_success_ratio")) {
        worstChannelsSuccessRatio = Double.parseDouble(element.getText());
      }
      if (element.getName().equals("best_channels_success_ratio")) {
        bestChannelsSuccessRatio = Double.parseDouble(element.getText());
      }
    }
    return true;
  }
}
