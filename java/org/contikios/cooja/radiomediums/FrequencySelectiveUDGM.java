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
  private double worstChannelsBitErrorRate = 0.001d;
  private double bestChannelsBitErrorRate = 0.00001d;
  private Map<Radio, Map<Radio, double[]>> edgeToBitErrorRates;

  public FrequencySelectiveUDGM(Simulation simulation) {
    super(simulation);
    edgeToBitErrorRates = new HashMap<>();
  }

  @Override
  public double getTxSuccessProbability() {
    return 1.0d;
  }

  private double[] generateBitErrorRates() {
    Random random = simulation.getRandomGenerator();
    double bitErrorRates[] = new double[CHANNELS_COUNT];
    Set<Integer> setIndices = new HashSet<>();
    double nextBitErrorRate = bestChannelsBitErrorRate;
    double stepSize = (worstChannelsBitErrorRate - bestChannelsBitErrorRate)
        / (CHANNELS_COUNT - 1);

    while(setIndices.size() < CHANNELS_COUNT) {
      int index;
      do {
        index = random.nextInt(CHANNELS_COUNT);
      } while(setIndices.contains(index));
      setIndices.add(index);
      bitErrorRates[index] = nextBitErrorRate;
      nextBitErrorRate += stepSize;
    }
    return bitErrorRates;
  }

  private Map<Radio, double[]> getOrCreateBitErrorRatesMap(Radio radio) {
    Map<Radio, double[]> bitErrorRates = edgeToBitErrorRates.get(radio);
    if (bitErrorRates == null) {
      bitErrorRates = new HashMap<Radio, double[]>();
      edgeToBitErrorRates.put(radio, bitErrorRates);
    }
    return bitErrorRates;
  }

  private double[] getOrCreateBitErrorRatesBetween(Radio a, Radio b) {
    Map<Radio, double[]> asBitErrorRates = getOrCreateBitErrorRatesMap(a);
    Map<Radio, double[]> bsBitErrorRates = getOrCreateBitErrorRatesMap(b);

    double[] bitErrorRates = asBitErrorRates.get(b);
    if(bitErrorRates != null) {
      return bitErrorRates;
    }

    bitErrorRates = bsBitErrorRates.get(a);
    if(bitErrorRates != null) {
      return bitErrorRates;
    }

    bitErrorRates = generateBitErrorRates();
    asBitErrorRates.put(b, bitErrorRates);
    return bitErrorRates;
  }

  @Override
  public double getRxSuccessProbability(Radio source, Radio dest) {
    if(super.getRxSuccessProbability(source, dest) == 0.0d) {
      /* not in range */
      return 0.0d;
    }

    double[] bitErrorRates = getOrCreateBitErrorRatesBetween(source, dest);
    int packetLength = source.getLastPacketTransmitted().getPacketData().length;
    double bitErrorRate = bitErrorRates[source.getChannel() - MIN_CHANNEL];
    return Math.pow(1 - bitErrorRate, packetLength * 8);
  }

  public void deteriorateLink(Radio a, Radio b) {
    double[] bitErrorRates = getOrCreateBitErrorRatesBetween(a, b);
    for (int i = 0; i < bitErrorRates.length; i++) {
      bitErrorRates[i] *= 10;
    }
  }

  public void improveLink(Radio a, Radio b) {
    double[] bitErrorRates = getOrCreateBitErrorRatesBetween(a, b);
    for (int i = 0; i < bitErrorRates.length; i++) {
      bitErrorRates[i] /= 10;
    }
  }

  @Override
  public Collection<Element> getConfigXML() {
    Collection<Element> config = super.getConfigXML();
    Element element;

    element = new Element("worst_channels_bit_error_rate");
    element.setText(Double.toString(worstChannelsBitErrorRate));
    config.add(element);

    element = new Element("best_channels_bit_error_rate");
    element.setText(Double.toString(bestChannelsBitErrorRate));
    config.add(element);

    return config;
  }

  @Override
  public boolean setConfigXML(Collection<Element> configXML, boolean visAvailable) {
    super.setConfigXML(configXML, visAvailable);
    for (Element element : configXML) {
      if (element.getName().equals("worst_channels_bit_error_rate")) {
        worstChannelsBitErrorRate = Double.parseDouble(element.getText());
      }
      if (element.getName().equals("best_channels_bit_error_rate")) {
        bestChannelsBitErrorRate = Double.parseDouble(element.getText());
      }
    }
    return true;
  }
}
