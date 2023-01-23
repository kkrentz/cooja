/*
 * Copyright (c) 2008, Swedish Institute of Computer Science.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
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
 * This file is part of MSPSim.
 *
 * $Id$
 *
 * -----------------------------------------------------------------
 *
 * ConfigManager
 *
 * Author  : Joakim Eriksson, Niclas Finne
 * Created : Fri Oct 11 15:24:14 2002
 * Updated : $Date$
 *           $Revision$
 */
package se.sics.mspsim.util;

import java.io.File;
import java.io.FileNotFoundException;
import java.io.IOException;
import java.io.PrintStream;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.util.Properties;
import java.util.StringTokenizer;

public class ConfigManager {

  protected Properties properties = null;


  // -------------------------------------------------------------------
  // Config file handling
  // -------------------------------------------------------------------

  public boolean loadConfiguration(String configFile) {
    try (var input = Files.newBufferedReader(new File(configFile).toPath(), StandardCharsets.UTF_8)) {
      var p = new Properties();
      p.load(input);
      this.properties = p;
      return true;
    } catch (FileNotFoundException e) {
      return false;
    } catch (IOException e) {
      throw new IllegalArgumentException("could not read config file '" + configFile + "': " + e);
    }
  }

  // -------------------------------------------------------------------
  // Properties handling
  // -------------------------------------------------------------------

  /**
   * Returns the property names. Does not include inherited properties.
   *
   * @return an array with the non-inherited property names
   */
  public String[] getPropertyNames() {
    if (properties == null) {
      return new String[0];
    }
    synchronized (properties) {
      return properties.keySet().toArray(new String[0]);
    }
  }

  public String getProperty(String name) {
    return getProperty(name, null);
  }

  public String getProperty(String name, String defaultValue) {
    String value = (properties != null)
    ? properties.getProperty(name)
        : null;

    if (value == null || value.isEmpty()) {
      return defaultValue;
    }
    return value;
  }

  public void setProperty(String name, String value) {
    if (properties == null) {
      synchronized (this) {
        if (properties == null) {
          properties = new Properties();
        }
      }
    }

    if (value == null) {
      properties.remove(name);
    } else {
      properties.put(name, value);
    }
  }

  public String[] getPropertyAsArray(String name) {
    String valueList = getProperty(name, null);
    if (valueList != null) {
      StringTokenizer tok = new StringTokenizer(valueList, ", \t");
      int len = tok.countTokens();
      if (len > 0) {
        String[] values = new String[len];
        for (int i = 0; i < len; i++) {
          values[i] = tok.nextToken();
        }
        return values;
      }
    }
    return null;
  }

  public int getPropertyAsInt(String name, int defaultValue) {
    String value = getProperty(name, null);
    if (value == null) {
      return defaultValue;
    }
    try {
      return Integer.parseInt(value);
    } catch (Exception e) {
      System.err.println("config '" + name + "' has a non-integer value '" + value + '\'');
    }
    return defaultValue;
  }

  public int[] getPropertyAsIntArray(String name) {
    String valueList = getProperty(name, null);
    if (valueList != null) {
      StringTokenizer tok = new StringTokenizer(valueList, ", \t/");
      int len = tok.countTokens();
      if (len > 0) {
        try {
          int[] values = new int[len];
          for (int i = 0; i < len; i++) {
            values[i] = Integer.parseInt(tok.nextToken());
          }
          return values;
        } catch (NumberFormatException e) {
          // Ignore parse errors and try secondary value if specified and not already tried
        }
      }
    }
    return null;
  }

  public long getPropertyAsLong(String name, long defaultValue) {
    String value = getProperty(name, null);
    if (value == null) {
      return defaultValue;
    }
    try {
      return Long.parseLong(value);
    } catch (Exception e) {
      System.err.println("config '" + name + "' has a non-long value '" + value + '\'');
    }
    return defaultValue;
  }

  public float getPropertyAsFloat(String name, float defaultValue) {
    String value = getProperty(name, null);
    if (value == null) {
      return defaultValue;
    }
    try {
      return Float.parseFloat(value);
    } catch (Exception e) {
      System.err.println("config '" + name + "' has a non-float value '" + value + '\'');
    }
    return defaultValue;
  }

  public double getPropertyAsDouble(String name, double defaultValue) {
    String value = getProperty(name, null);
    if (value == null) {
      return defaultValue;
    }
    try {
      return Double.parseDouble(value);
    } catch (Exception e) {
      System.err.println("config '" + name + "' has a non-double value '" + value + '\'');
    }
    return defaultValue;
  }

  public boolean getPropertyAsBoolean(String name, boolean defaultValue) {
    String value = getProperty(name, null);
    return value == null ? defaultValue : "true".equals(value) || "yes".equals(value) || "1".equals(value);
  }

  public void print(PrintStream out) {
      properties.list(out);
  }


} // ConfigManager
